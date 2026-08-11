#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <core/mode.h>
#include <core/trace_stats.h>
#include <eval/quality.h>
#include <eval/rayset.h>
#include <eval/trace.h>
#include <reference/brute_force.h>
#include <util/camera.h>
#include <util/image.h>
#include <util/log.h>
#include <util/mesh.h>
#include <util/metrics.h>
#include <util/parallel.h>
#include <util/stats.h>
#include <util/timer.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace bvh;

namespace 
{
	class args
	{
	public :
		args(int argc, char** argv)
		{
			for (int i = 1; i < argc; ++i)
			{
				std::string a = argv[i];
				if (a.rfind("--", 0) != 0)
				{
					LOG_WARN("ignoring positional argument '%s'", a.c_str());
					continue;
				}
				a = a.substr(2);
				const size_t eq = a.find("=");
				if (eq == std::string::npos) _values[a] = "1";
				else						 _values[a.substr(0, eq)] = a.substr(eq + 1);
			}
		}

		std::string get(const char* key, const char* fallback) const
		{
			auto it = _values.find(key);
			return it == _values.end() ? std::string(fallback) : it->second;
		}

		u32 get_u32(const char* key, u32 fallback) const
		{
			auto it = _values.find(key);
			return it == _values.end() ? fallback : u32(std::strtoul(it->second.c_str(), nullptr, 10));
		}

		f32 get_f32(const char* key, f32 fallback) const
		{
			auto it = _values.find(key);
			return it == _values.end() ? fallback : f32(std::atof(it->second.c_str()));
		}

		bool has(const char* key) const { return _values.count(key) != 0; }

	private:
		std::map<std::string, std::string> _values;
	};

	void ensure_parent_dir(const std::string& path)
	{
		const std::filesystem::path p(path);
		if (p.has_parent_path() && !p.parent_path().empty())
			std::filesystem::create_directories(p.parent_path());
	}

	bool validate_against_oracle(const bvh2& tree, const mesh& m, const camera& cam, u32 stride)
	{
		const bvh2_view  view = tree.view();
		const auto prims = make_prims(m, tree);

		u64 checked = 0, mismatches = 0, ties = 0;
		f32 worst = 0.0f;
		std::vector<u32> scratch;

		for (u32 j = 0; j < cam.height(); j += stride)
		{
			for (u32 i = 0; i < cam.width(); i += stride)
			{
				const ray r = cam.generate_ray_through_pixel(i, j);

				hit fast;
				null_stats s1;
				intersect(view, r, fast, prims, s1);

				// The BVH reports a slot; the oracle reports a mesh triangle id.
				const u32 fast_prim = fast.valid() ? tree.prim_index(fast.id) : invalid_id;

				// Tie-aware (PLAN.md �6.1): an exact id match is only required when
				// the nearest hit is unique.
				const oracle_result cmp =
					compare_against_oracle(m, r, fast.valid(), fast.t, fast_prim, scratch);

				++checked;
				if (cmp.tie_resolved) ++ties;
				if (!cmp.agree)
				{
					++mismatches;
					if (cmp.t_delta > worst) worst = cmp.t_delta;
				}
			}
		}

		if (mismatches)
		{
			LOG_ERROR("  ORACLE MISMATCH: %llu/%llu rays differ (worst |dt| = %g)",
				(unsigned long long)mismatches, (unsigned long long)checked, worst);
			return false;
		}

		LOG_INFO("  oracle: %llu/%llu rays agree (%llu resolved by tie set)",
			(unsigned long long)checked, (unsigned long long)checked,
			(unsigned long long)ties);
		return true;
	}

	void print_usage()
	{
		printf(
			"trax CPU tracer\n"
			"\n"
			"  --scene=<path.obj>     mesh to load        (default scenes/bunny.obj)\n"
			"  --scale=<f>            uniform mesh scale  (default 1.0)\n"
			"  --width=<n>            image width         (default 512)\n"
			"  --height=<n>           image height        (default 512)\n"
			"  --out=<path.png>       image output        (default results/d1_normals.png)\n"
			"  --csv=<path.csv>       metrics row output  (default results/d1.csv)\n"
			"  --threads=<n>          0 = hardware        (default 0)\n"
			"  --verbose              verbose logging\n"
			"  --help\n");
	}
} // namespace

int main(int argc, char** argv)
{
	const args opts(argc, argv);
	if (opts.has("help")) { print_usage(); return 0; }

	log_init(opts.has("verbose") ? log_level::verbose : log_level::info);

	const std::string scene_path = opts.get("scene", "scenes/teapot.obj");
	const std::string out_dir = opts.get("out", "results");
	const std::string csv_path = opts.get("csv", "results/m1_bvh2.csv");
	const std::string workload_csv = opts.get("workload_csv", "results/m1_workload.csv");
	const u32 width = opts.get_u32("width", 512);
	const u32 height = opts.get_u32("height", 512);
	const u32 bins = opts.get_u32("bins", 32);
	const u32 threads = opts.get_u32("threads", 0);
	const f32 scale = opts.get_f32("scale", 1.0f);
	const bool want_epo = opts.has("epo");
	const bool validate = !opts.has("no-validate");
	const std::string rayset_dir = opts.get("raysets", "raysets");
	const u32 incoherent_count = opts.get_u32("rays", 262144);
	const u32 reps = opts.get_u32("reps", 3);
	const std::string wide_csv = opts.get("wide_csv", "results/m3_wide.csv");
	const bool run_wide = opts.has("wide");

	mesh original;
	if (!original.load_obj(scene_path, scale))
	{
		LOG_ERROR("could not load '%s'", scene_path.c_str());
		return 1;
	}

	const camera cam = camera::frame_bounds(original.bounds(), width, height);
	const std::string scene_name = std::filesystem::path(scene_path).filename().string();

	ensure_parent_dir(csv_path);
	ensure_parent_dir(workload_csv);
	ensure_parent_dir(wide_csv);
	ensure_parent_dir(out_dir + "/x");

	// baseline: brute force
	{
		image img(width, height);
		const render_result bf = render_normals(original, cam, img, threads);
		img.write_png(out_dir + "/m1_reference.png");

		LOG_INFO("brute force: %.3f s, %.3f MRays/s (%u tris/ray)",
			bf.seconds, bf.mrays_per_second(), original.triangle_count());

		metrics row;
		row.set("scene", scene_name);
		row.set("frame", 0u);
		row.set("triangles", original.triangle_count());
		row.set("builder", "brute_force");
		row.set("layout", "none");
		row.set("codec", "indexed_tri");
		row.set("maintenance", "static");
		row.set("traversal", "brute_force");
		row.set("ray_set", "primary");
		row.set("device", "cpu");
		row.set("mode", default_mode::name);
		row.set("bins", 0u);
		row.set("build_ms", 0.0, 2);
		row.set("nodes", 0u);
		row.set("leaves", 0u);
		row.set("max_depth", 0u);
		row.set("mean_leaf", 0.0, 2);
		row.set("bytes_per_tri", 0.0, 2);
		row.set("sah_cost", 0.0, 4);
		row.set("sah_cost_arches", 0.0, 2);
		row.set("epo", 0.0, 4);
		row.set("combined", 0.0, 4);
		row.set("node_steps_per_ray", 0.0, 3);
		row.set("prim_steps_per_ray", 0.0, 3);
		row.set("tri_tests_per_ray", double(original.triangle_count()), 3);
		row.set("max_stack", 0u);
		row.set("trace_s", bf.seconds, 4);
		row.set("mrays_s", bf.mrays_per_second(), 4);
		row.set("hit_rate", bf.rays ? double(bf.hits) / double(bf.rays) : 0.0, 4);
		row.set("oracle", "reference");
		row.set("evidence", "E2");
		row.flush(csv_path);
	}

	const split_method methods[] = {
		split_method::median,
		split_method::binned_sah_arches,
		split_method::binned_sah,
		split_method::sweep_sah,
	};

	bool all_valid = true;

	for (split_method method : methods)
	{
		LOG_INFO("--- %s ---", to_string(method));
		mesh m = original;

		build_args ba;
		ba.method = method;
		ba.bins = bins;

		bvh2 tree;
		tree.build(m, ba);
		tree.apply_reorder(m);
		tree.refit(m);

		if (tree.report().max_depth + 2 >= bvh2_stack_size)
		{
			LOG_ERROR("  tree depth %u exceeds the %u-entry traversal stack",
				tree.report().max_depth, bvh2_stack_size);
			all_valid = false;
			continue;
		}

		bool valid = true;
		if (validate) valid = validate_against_oracle(tree, m, cam, 8);
		all_valid &= valid;

		quality_args qa;
		qa.compute_epo = want_epo;
		timer qt;
		const quality_metrics q = evaluate(tree, m, qa);
		const double quality_ms = qt.elapsed_ms();

		image             img(width, height);
		std::vector<u32>  counts;
		const trace_result tr = render_bvh2(tree, m, cam, img, &counts, threads);

		const std::string tag = to_string(method);
		img.write_png(out_dir + "/m1_" + tag + ".png");
		image::from_counts(counts, width, height).write_png(out_dir + "/m1_" + tag + "_heatmap.png", false);

		LOG_INFO("  SAH %.4f  EPO %.4f  nodes %u  depth %u  %.3f MRays/s  %.2f node steps/ray",
			q.sah_cost, q.epo, q.node_count, q.max_depth,
			tr.mrays_per_second(), tr.node_steps_per_ray());

		rayset_args ra;
		ra.width = width;
		ra.height = height;
		ra.incoherent_count = incoherent_count;

		for (ray_distribution dist : all_ray_distributions)
		{
			rayset rs;
			rs.scene = scene_name;
			if (!rayset::load_or_generate(rs, rayset_dir, dist, m, tree, cam, ra))
			{
				LOG_ERROR("  could not obtain ray set '%s'", to_string(dist));
				continue;
			}
			if (rs.empty()) continue;

			const bool any_hit = (dist == ray_distribution::shadow_ao);
			const trace_result rr = any_hit ? occlude_rayset(tree, m, rs, threads, reps)
				: trace_rayset(tree, m, rs, threads, reps);

			LOG_INFO("    %-11s %7u rays  %6.2f node steps/ray  %6.2f tri tests/ray  %.3f MRays/s",
				to_string(dist), rs.size(), rr.node_steps_per_ray(),
				rr.tri_tests_per_ray(), rr.mrays_per_second());

			metrics rr_row;
			rr_row.set("scene", scene_name);
			rr_row.set("frame", 0u);
			rr_row.set("triangles", m.triangle_count());
			rr_row.set("builder", tag);
			rr_row.set("layout", "bvh2");
			rr_row.set("codec", "indexed_tri");
			rr_row.set("maintenance", "static");
			rr_row.set("traversal", any_hit ? "ordered_anyhit" : "ordered_closest");
			rr_row.set("ray_set", to_string(dist));
			rr_row.set("device", "cpu");
			rr_row.set("mode", default_mode::name);
			rr_row.set("rayset_hash", std::to_string(rs.hash()));
			rr_row.set("rays", i64(rr.rays));
			rr_row.set("hits", i64(rr.hits));
			rr_row.set("node_steps_per_ray", rr.node_steps_per_ray(), 3);
			rr_row.set("prim_steps_per_ray", rr.prim_steps_per_ray(), 3);
			rr_row.set("tri_tests_per_ray", rr.tri_tests_per_ray(), 3);
			rr_row.set("max_stack", rr.max_stack);
			rr_row.set("trace_s", rr.seconds, 4);
			rr_row.set("mrays_s", rr.mrays_per_second(), 4);
			rr_row.set("source", "S-cpu");
			rr_row.set("reps", reps);
			rr_row.set("trace_s_min", rr.seconds_min, 4);
			rr_row.set("trace_s_stddev", rr.seconds_stddev, 5);
			rr_row.set("trace_s_cv", rr.cv(), 4);
			rr_row.flush(workload_csv);
		}
		if (want_epo) LOG_INFO("  (quality eval took %.0f ms)", quality_ms);

		metrics row;
		row.set("scene", scene_name);
		row.set("frame", 0u);
		row.set("triangles", m.triangle_count());
		row.set("builder", tag);
		row.set("layout", "bvh2");
		row.set("codec", "indexed_tri");
		row.set("maintenance", "static");
		row.set("traversal", "ordered_scalar");
		row.set("ray_set", "primary");
		row.set("device", "cpu");
		row.set("mode", default_mode::name);
		row.set("bins", (method == split_method::binned_sah || method == split_method::binned_sah_arches) ? bins : 0u);
		row.set("build_ms", tree.report().build_ms, 2);
		row.set("nodes", q.node_count);
		row.set("leaves", q.leaf_count);
		row.set("max_depth", q.max_depth);
		row.set("mean_leaf", q.mean_leaf_size, 2);
		row.set("bytes_per_tri", q.bytes_per_tri, 2);
		row.set("sah_cost", q.sah_cost, 4);
		row.set("sah_cost_arches", q.sah_cost_arches, 2);
		// Blank, not 0, when EPO was not requested. A literal 0.0000 is
		// indistinguishable from a tree that genuinely has no overlap.
		if (want_epo) { row.set("epo", q.epo, 4); row.set("combined", q.combined, 4); }
		else          { row.set("epo", ""); row.set("combined", ""); }
		row.set("node_steps_per_ray", tr.node_steps_per_ray(), 3);
		row.set("prim_steps_per_ray", tr.prim_steps_per_ray(), 3);
		row.set("tri_tests_per_ray", tr.tri_tests_per_ray(), 3);
		row.set("max_stack", tr.max_stack);
		row.set("trace_s", tr.seconds, 4);
		row.set("mrays_s", tr.mrays_per_second(), 4);
		row.set("hit_rate", tr.rays ? double(tr.hits) / double(tr.rays) : 0.0, 4);
		row.set("oracle", valid ? "match" : "MISMATCH");
		row.set("evidence", "E1+E2");
		row.flush(csv_path);
		row.print(stdout);
	}

	// width x collapse method
	if (run_wide)
	{
		const split_method base = split_method::binned_sah;

		for (u32 w : {2u, 4u, 8u})
		{
			for (collapse_method cm : {collapse_method::greedy, collapse_method::dynamic_programming})
			{
				if (w == 2 && cm == collapse_method::dynamic_programming) continue; // width 2 is the uncollapsed tree

				mesh m = original;

				build_args ba;
				ba.method = base;
				ba.bins = bins;
				ba.silent = true;

				bvh2 tree;
				tree.build(m, ba);
				tree.apply_reorder(m);

				collapse_report cr{};
				cr.width = 2;
				if (w > 2)
				{
					collapse_args ca;
					ca.width = w;
					ca.method = cm;
					ca.silent = true;
					cr = collapse(tree, m, ca);
				}
				else
				{
					cr = collapse_report{};
					cr.width = 2;
					cr.node_count = tree.report().node_count;
					cr.interior_count = tree.report().interior_count;
					cr.leaf_count = tree.report().leaf_count;
					cr.max_depth = tree.report().max_depth;
					cr.mean_fullness = 2.0;
				}

				if (tree.report().max_depth + 2 >= bvh2_stack_size)
				{
					LOG_ERROR("  w%u tree depth %u exceeds the traversal stack", w, tree.report().max_depth);
					continue;
				}

				const quality_metrics q = evaluate(tree, m);
				const overlap_profile op = compute_overlap_profile(tree);

				LOG_INFO("w%u %-6s: %u nodes, depth %u, fullness %.2f, SAH %.4f",
					w, w == 2 ? "-" : to_string(cm), cr.node_count, cr.max_depth,
					cr.mean_fullness, q.sah_cost);

				for (ray_distribution dist : all_ray_distributions)
				{
					rayset rs;
					rs.scene = scene_name;
					rayset_args ra;
					ra.width = width; ra.height = height;
					ra.incoherent_count = incoherent_count;
					if (!rayset::load_or_generate(rs, rayset_dir, dist, m, tree, cam, ra)) continue;
					if (rs.empty()) continue;

					const bool any_hit = (dist == ray_distribution::shadow_ao);
					const trace_result rr = any_hit ? occlude_rayset(tree, m, rs, threads, reps)
						: trace_rayset(tree, m, rs, threads, reps);

					metrics row;
					row.set("scene", scene_name);
					row.set("builder", to_string(base));
					row.set("layout", w == 2 ? "bvh2" : (w == 4 ? "bvh4" : "bvh8"));
					row.set("width", w);
					row.set("collapse", w == 2 ? "none" : to_string(cm));
					row.set("codec", "indexed_tri");
					row.set("maintenance", "static");
					row.set("ray_set", to_string(dist));
					row.set("device", "cpu");
					row.set("mode", default_mode::name);
					row.set("nodes", cr.node_count);
					row.set("interior", cr.interior_count);
					row.set("max_depth", cr.max_depth);
					row.set("mean_fullness", cr.mean_fullness, 3);
					row.set("collapse_ms", cr.collapse_ms, 2);
					row.set("sah_cost", q.sah_cost, 4);
					// The one to compare ACROSS widths; sah_cost charges per node
					// and so falls with width for free.
					row.set("sah_cost_slots", q.sah_cost_slots, 4);
					row.set("bytes_per_tri", q.bytes_per_tri, 2);
					row.set("overlap_d0", op.mean_overlap[0], 4);
					row.set("overlap_d3", op.mean_overlap[3], 4);
					row.set("overlap_d6", op.mean_overlap[6], 4);
					// node steps fall with width; box tests per step RISE. The
					// trade this milestone exists to measure.
					row.set("node_steps_per_ray", rr.node_steps_per_ray(), 3);
					row.set("box_tests_per_ray", rr.rays ? double(rr.box_tests) / double(rr.rays) : 0.0, 3);
					row.set("tri_tests_per_ray", rr.tri_tests_per_ray(), 3);
					row.set("max_stack", rr.max_stack);
					row.set("trace_s", rr.seconds, 4);
					row.set("trace_s_cv", rr.cv(), 4);
					row.set("mrays_s", rr.mrays_per_second(), 4);
					row.set("source", "S-cpu");
					row.set("reps", reps);
					row.flush(wide_csv);
				}
			}
		}
	}

	report_thread_stats();
	print_stats(stdout);

	if (!all_valid) LOG_ERROR("one or more trees disagreed with the oracle");
	else            LOG_INFO("all trees agree with the brute-force oracle");

	log_shutdown();
	return all_valid ? 0 : 1;
}

