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

#include "direction_d.h"
#include "phase0.h"
#include "geometry_collapse.h"

#include <chrono>
#include <ctime>
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

				const u32 fast_prim = fast.valid() ? tree.prim_index(fast.id) : invalid_id;
				const oracle_result cmp = compare_against_oracle(m, r, fast.valid(), fast.t, fast_prim, scratch);

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

	bool validate_rayset(const bvh2& tree, const mesh& m, const rayset& rs, bool any_hit,
	                     const char* tag, u32 max_report = 5)
	{
		const bvh2_view view  = tree.view();
		const auto      prims = make_prims(m, tree);

		u64 mismatches = 0, ties = 0;
		std::vector<u32> scratch;

		for (u32 i = 0; i < rs.size(); ++i)
		{
			const ray r = rs.get(i);

			if (any_hit)
			{
				null_stats s1, s2;
				const bool fast = occluded(view, r, prims, s1);
				const bool slow = occluded_brute_force(m, r, s2);
				if (fast == slow) continue;

				++mismatches;
				if (mismatches <= max_report)
					LOG_ERROR("  ORACLE [%s] anyhit rayset=%llu ray=%u: bvh=%d oracle=%d "
					          "o=(%g %g %g) d=(%g %g %g) t_max=%g",
					          tag, (unsigned long long)rs.hash(), i, int(fast), int(slow),
					          r.o.x, r.o.y, r.o.z, r.d.x, r.d.y, r.d.z, r.t_max);
				continue;
			}

			hit h;
			h.t = r.t_max;
			null_stats s;
			intersect(view, r, h, prims, s);

			const u32 id = h.valid() ? tree.prim_index(h.id) : invalid_id;
			const oracle_result cmp = compare_against_oracle(m, r, h.valid(), h.t, id, scratch);

			if (cmp.tie_resolved) ++ties;
			if (cmp.agree) continue;

			++mismatches;
			if (mismatches <= max_report)
				LOG_ERROR("  ORACLE [%s] closest rayset=%llu ray=%u: hit=%d t=%g id=%u "
				          "(%s%s%s) |dt|=%g tie_set=%zu",
				          tag, (unsigned long long)rs.hash(), i, int(h.valid()), h.t, id,
				          cmp.miss_mismatch ? "hit/miss " : "",
				          cmp.t_mismatch ? "t " : "",
				          cmp.id_mismatch ? "id" : "",
				          cmp.t_delta, scratch.size());
		}

		if (mismatches)
		{
			LOG_ERROR("  [%s] %llu/%u rays disagree with the oracle", tag,
			          (unsigned long long)mismatches, rs.size());
			return false;
		}
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
			"  --run_id=<id>          run identity        (default run_<date>_<us>)\n"
			"  --out=<dir>            image output dir    (default results/<run_id>)\n"
			"  --csv=<path.csv>       builder summary     (default results/<run_id>/m1_bvh2.csv)\n"
			"  --workload_csv=<path>  per-ray-set rows    (default results/<run_id>/m1_workload.csv)\n"
			"  --wide_csv=<path>      wide topology rows  (default results/<run_id>/m3_wide.csv)\n"
			"  --depth_csv=<path>     overlap profile     (default results/<run_id>/m3_depth_profile.csv)\n"
			"  --threads=<n>          0 = hardware        (default 0)\n"
			"  --direction_d          also run the Direction D mechanism pass\n"
			"  --direction_d_only     run ONLY Direction D, nothing else\n"
			"  --phase0               run ONLY the Phase 0 feasibility probe\n"
			"  --geometry_collapse    run ONLY the geometry-derived BVH8 collapse experiment\n"
			"  --cameras=<path.csv>   frozen evaluation cameras\n"
			"  --phase0_candidates=<n>  camera candidates to screen  (default 64)\n"
			"  --phase0_screen_res=<n>  camera screening resolution  (default 64)\n"
			"  --direction_d_validation_rays=<n>  sampled oracle rays (0 = all)\n"
			"  --git_commit=<sha>     recorded verbatim into every Direction D row\n"
			"  --dirty=<0|1>          recorded verbatim into every Direction D row\n"
			"  --verbose              verbose logging\n"
			"  --help\n");
	}
} // namespace

int main(int argc, char** argv)
{
	const args opts(argc, argv);
	if (opts.has("help")) { print_usage(); return 0; }

	log_init(opts.has("verbose") ? log_level::verbose : log_level::info);

	// Run identity, so rows from different invocations are never silently mixed.
	// Microsecond resolution: a seconds stamp collides whenever two runs start in
	// the same second, which merges two experiments into one directory.
	char run_id_buf[64];
	{
		const auto now = std::chrono::system_clock::now();
		const long long us = (long long)std::chrono::duration_cast<std::chrono::microseconds>(
			now.time_since_epoch()).count();
		const std::time_t secs = (std::time_t)(us / 1000000);
		std::tm tmv{};
#if defined(_WIN32)
		localtime_s(&tmv, &secs);
#else
		localtime_r(&secs, &tmv);
#endif
		char stamp[32];
		std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);
		snprintf(run_id_buf, sizeof(run_id_buf), "run_%s_%06lld", stamp, us % 1000000);
	}
	const std::string run_id = opts.get("run_id", run_id_buf);

	// Every artifact defaults beneath results/<run_id>/ so no two runs can
	// overwrite each other. Explicit --out/--csv/--workload_csv/--wide_csv/
	// --depth_csv overrides are still honoured verbatim.
	const std::string run_dir = "results/" + run_id;

	const std::string scene_path = opts.get("scene", "scenes/teapot.obj");
	const std::string out_dir = opts.get("out", run_dir.c_str());
	const std::string csv_path = opts.get("csv", (run_dir + "/m1_bvh2.csv").c_str());
	const std::string workload_csv = opts.get("workload_csv", (run_dir + "/m1_workload.csv").c_str());
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
	const std::string wide_csv = opts.get("wide_csv", (run_dir + "/m3_wide.csv").c_str());
	const bool run_wide = opts.has("wide");
	const std::string depth_csv = opts.get("depth_csv", (run_dir + "/m3_depth_profile.csv").c_str());
	const bool run_direction_d_only = opts.has("direction_d_only");
	const bool run_phase0_only = opts.has("phase0");
	const bool run_geometry_collapse_only = opts.has("geometry_collapse");
	const bool run_node_terms_only = opts.has("node_terms");

	// The geometry-derived BVH8 collapse experiment loads the scene itself.
	if (run_geometry_collapse_only || run_node_terms_only)
	{
		geometry_collapse_args gc;
		gc.run_id = run_id;
		gc.run_dir = run_dir;
		gc.scene_path = scene_path;
		gc.scene_name = std::filesystem::path(scene_path).filename().string();
		gc.cameras_csv = opts.get("cameras", "experiments/wide_collapse/cameras.csv");
		gc.image_dir = opts.get("image_dir", "experiments/geometry_wide_collapse");
		gc.git_commit = opts.get("git_commit", "");
		gc.dirty = opts.get_u32("dirty", 0u) != 0u;
		gc.width = width;
		gc.height = height;
		gc.bins = bins;
		gc.threads = threads;

		const bool ok = run_node_terms_only ? run_node_terms(gc) : run_geometry_collapse(gc);
		report_thread_stats();
		print_stats(stdout);
		if (!ok) LOG_ERROR("geometry-derived BVH8 collapse reported a failure");
		else     LOG_INFO("geometry-derived BVH8 collapse complete");
		log_shutdown();
		return ok ? 0 : 1;
	}

	// Phase 0 loads the scene itself so it can time the load; it runs nothing else.
	if (run_phase0_only)
	{
		phase0_args p0;
		p0.run_id = run_id;
		p0.run_dir = run_dir;
		p0.scene_path = scene_path;
		p0.scene_name = std::filesystem::path(scene_path).filename().string();
		p0.git_commit = opts.get("git_commit", "");
		p0.dirty = opts.get_u32("dirty", 0u) != 0u;
		p0.screen_res = opts.get_u32("phase0_screen_res", 64u);
		p0.candidates = opts.get_u32("phase0_candidates", 64u);
		p0.oracle_probe_rays = opts.get_u32("phase0_oracle_rays", 0u);
		p0.threads = threads;

		const bool ok = run_phase0(p0);
		report_thread_stats();
		print_stats(stdout);
		if (!ok) LOG_ERROR("Phase 0 reported a failure");
		else     LOG_INFO("Phase 0 complete");
		log_shutdown();
		return ok ? 0 : 1;
	}

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
	ensure_parent_dir(depth_csv);
	ensure_parent_dir(out_dir + "/x");

	// A bare file name cannot say which run a profile belongs to; record the
	// path this run actually wrote.
	const std::string depth_csv_ref = std::filesystem::path(depth_csv).generic_string();

	// Direction D is an opt-in mechanism experiment. It never runs as part of an
	// ordinary M1/M3 invocation, and --direction_d_only runs nothing else.
	bool direction_d_ok = true;
	if (run_direction_d_only || opts.has("direction_d"))
	{
		direction_d_args dda;
		dda.run_id = run_id;
		dda.run_dir = run_dir;
		dda.scene_path = scene_path;
		dda.scene_name = scene_name;
		dda.git_commit = opts.get("git_commit", "");
		dda.dirty = opts.get_u32("dirty", 0u) != 0u;
		dda.width = width;
		dda.height = height;
		dda.bins = bins;
		dda.incoherent_count = incoherent_count;
		dda.threads = 1u; // frozen for the first experiment
		dda.validate = validate;
		dda.validation_samples = opts.get_u32("direction_d_validation_rays", 0u);

		direction_d_ok = run_direction_d(original, dda);

		if (run_direction_d_only)
		{
			report_thread_stats();
			print_stats(stdout);
			if (!direction_d_ok) LOG_ERROR("Direction D failed");
			else                 LOG_INFO("Direction D complete");
			log_shutdown();
			return direction_d_ok ? 0 : 1;
		}
	}

	// baseline: brute force
	{
		image img(width, height);
		const render_result bf = render_normals(original, cam, img, threads);
		img.write_png(out_dir + "/m1_reference.png");

		LOG_INFO("brute force: %.3f s, %.3f MRays/s (%u tris/ray)",
			bf.seconds, bf.mrays_per_second(), original.triangle_count());

		metrics row;
		row.set("run_id", run_id);
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
		row.set("node_bytes_per_tri", 0.0, 2);
		row.set("sah_node_cost", 0.0, 4);
		row.set("sah_cost_arches", 0.0, 2);
		row.set("epo", 0.0, 4);
		row.set("combined", 0.0, 4);
		row.set("node_steps_per_ray", 0.0, 3);
		row.set("prim_steps_per_ray", 0.0, 3);
		row.set("tri_tests_per_ray", double(original.triangle_count()), 3);
		row.set("max_stack", 0u);
		row.set("render_s", bf.seconds, 4);
		row.set("render_mrays_s", bf.mrays_per_second(), 4);
		row.set("hit_rate", bf.rays ? double(bf.hits) / double(bf.rays) : 0.0, 4);
		row.set("oracle", "reference");
		row.set("source_analytic", "");
		row.set("source_timing", "S-cpu");
		row.flush(csv_path);
	}

	const split_method methods[] = {
		split_method::median,
		split_method::binned_sah_arches,
		split_method::binned_sah,
		split_method::sweep_sah,
	};

	bool all_valid = direction_d_ok;

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
			char rs_tag[96];
			snprintf(rs_tag, sizeof(rs_tag), "%s/%s", tag.c_str(), to_string(dist));

			// A ray set we could not obtain is a failed run, not something to
			// skip quietly: the surviving rows would misrepresent the workload.
			if (!rayset::load_or_generate(rs, rayset_dir, dist, m, tree, cam, ra) || rs.empty())
			{
				LOG_ERROR("  could not obtain ray set '%s'", rs_tag);
				all_valid = false;
				continue;
			}

			const bool any_hit = (dist == ray_distribution::shadow_ao);

			// Validate the COMPLETE ray set before timing or publishing it: the
			// camera-lattice check above only covers primary rays.
			if (validate && !validate_rayset(tree, m, rs, any_hit, rs_tag))
			{
				all_valid = false;
				continue; // omit the row entirely rather than publish it
			}

			const trace_result rr = any_hit ? occlude_rayset(tree, m, rs, threads, reps)
				: trace_rayset(tree, m, rs, threads, reps);

			LOG_INFO("    %-11s %7u rays  %6.2f node steps/ray  %6.2f tri tests/ray  %.3f MRays/s",
				to_string(dist), rs.size(), rr.node_steps_per_ray(),
				rr.tri_tests_per_ray(), rr.mrays_per_second());

			metrics rr_row;
			rr_row.set("run_id", run_id);
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
			rr_row.set("threads_requested", threads);
			rr_row.flush(workload_csv);
		}
		if (want_epo) LOG_INFO("  (quality eval took %.0f ms)", quality_ms);

		metrics row;
		row.set("run_id", run_id);
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
		row.set("node_bytes_per_tri", q.bytes_per_tri, 2);
		row.set("sah_node_cost", q.sah_cost, 4);
		row.set("sah_cost_arches", q.sah_cost_arches, 2);
		if (want_epo) { row.set("epo", q.epo, 4); row.set("combined", q.combined, 4); }
		else          { row.set("epo", ""); row.set("combined", ""); }
		row.set("node_steps_per_ray", tr.node_steps_per_ray(), 3);
		row.set("prim_steps_per_ray", tr.prim_steps_per_ray(), 3);
		row.set("tri_tests_per_ray", tr.tri_tests_per_ray(), 3);
		row.set("max_stack", tr.max_stack);
		// These come from render_bvh2: a whole frame including primary ray
		// generation and image writes, NOT a pure traversal timing.
		row.set("render_s", tr.seconds, 4);
		row.set("render_mrays_s", tr.mrays_per_second(), 4);
		row.set("hit_rate", tr.rays ? double(tr.hits) / double(tr.rays) : 0.0, 4);
		row.set("oracle", valid ? "match" : "MISMATCH");
		row.set("source_analytic", "S-analytic");
			row.set("source_timing", "S-cpu");
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
					all_valid = false;
					continue;
				}

				const char* topology = w == 2 ? "bvh2" : (w == 4 ? "bvh4" : "bvh8");
				const char* collapse_name = w == 2 ? "none" : to_string(cm);

				// Phase 1: obtain and validate ALL six ray sets before a single
				// row is written. The depth-profile rows are analytic, but they
				// still describe this topology -- if the topology disagrees with
				// the oracle, no row of any kind may be left behind claiming to
				// describe it.
				rayset_args ra;
				ra.width = width; ra.height = height;
				ra.incoherent_count = incoherent_count;

				constexpr u32 dist_count = u32(sizeof(all_ray_distributions) / sizeof(all_ray_distributions[0]));
				rayset sets[dist_count];
				bool topology_ok = true;

				for (u32 i = 0; i < dist_count && topology_ok; ++i)
				{
					const ray_distribution dist = all_ray_distributions[i];
					rayset& rs = sets[i];
					rs.scene = scene_name;

					char tag[96];
					snprintf(tag, sizeof(tag), "w%u/%s/%s", w, collapse_name, to_string(dist));

					// A ray set we could not obtain is a failed run, not
					// something to skip quietly.
					if (!rayset::load_or_generate(rs, rayset_dir, dist, m, tree, cam, ra) || rs.empty())
					{
						LOG_ERROR("  could not obtain ray set '%s'", tag);
						topology_ok = false;
						break;
					}

					const bool any_hit = (dist == ray_distribution::shadow_ao);
					if (validate && !validate_rayset(tree, m, rs, any_hit, tag)) topology_ok = false;
				}

				if (!topology_ok)
				{
					all_valid = false;
					continue; // no depth rows and no timing rows for this topology
				}

				// Phase 2: the topology is trusted, so publish it.
				const quality_metrics q = evaluate(tree, m);
				const overlap_profile op = compute_overlap_profile(tree);

				for (u32 d = 0; d < op.depth_count; ++d)
				{
					const depth_overlap_stats& ds = op.depth[d];
					if (ds.internal_nodes == 0) continue;

					metrics dr;
					dr.set("run_id", run_id);
					dr.set("scene", scene_name);
					dr.set("topology", topology);
					dr.set("width", w);
					dr.set("collapse", collapse_name);
					dr.set("depth", d);
					dr.set("internal_nodes", ds.internal_nodes);
					dr.set("invalid_parent_nodes", ds.invalid_parent_nodes);
					dr.set("pair_count", i64(ds.pair_count));
					dr.set("mean_child_area_ratio", ds.mean_child_area_ratio, 5);
					dr.set("mean_pair_overlap", ds.mean_pair_overlap, 6);
					dr.set("p95_pair_overlap", ds.p95_pair_overlap, 6);
					dr.set("max_pair_overlap", ds.max_pair_overlap, 6);
					dr.set("sum_pair_overlap_width_dependent", ds.sum_pair_overlap, 6);
					dr.set("source", "S-analytic");
					dr.flush(depth_csv);
				}
				if (op.nodes_beyond_buckets)
					LOG_WARN("  %u nodes deeper than the %u depth buckets were not profiled",
						op.nodes_beyond_buckets, overlap_profile::max_depth_buckets);

				LOG_INFO("w%u %-6s: %u nodes, depth %u, fullness %.2f, SAH %.4f",
					w, w == 2 ? "-" : to_string(cm), cr.node_count, cr.max_depth,
					cr.mean_fullness, q.sah_cost);

				for (u32 i = 0; i < dist_count; ++i)
				{
					const ray_distribution dist = all_ray_distributions[i];
					const rayset& rs = sets[i];
					const bool any_hit = (dist == ray_distribution::shadow_ao);

					const trace_result rr = any_hit ? occlude_rayset(tree, m, rs, threads, reps)
						: trace_rayset(tree, m, rs, threads, reps);

					metrics row;
					row.set("run_id", run_id);
					row.set("scene", scene_name);
					row.set("builder", to_string(base));
					row.set("topology", topology);
					row.set("physical_layout", "slot32_aos");
					row.set("traversal_kernel", any_hit ? "scalar_storage_lifo" : "scalar_distance_sorted");
					row.set("width", w);
					row.set("collapse", collapse_name);
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
					row.set("sah_node_cost", q.sah_cost, 4);
					row.set("sah_slot_cost", q.sah_cost_slots, 4);
					row.set("node_bytes_per_tri", q.bytes_per_tri, 2);
					row.set("depth_profile_csv", depth_csv_ref);
					row.set("rayset_hash", std::to_string(rs.hash()));
					row.set("rays", i64(rr.rays));
					row.set("hits", i64(rr.hits));
					row.set("threads_requested", threads);
					row.set("node_steps_per_ray", rr.node_steps_per_ray(), 3);
					row.set("box_tests_per_ray", rr.rays ? double(rr.box_tests) / double(rr.rays) : 0.0, 3);
					row.set("tri_tests_per_ray", rr.tri_tests_per_ray(), 3);
					row.set("max_stack", rr.max_stack);
					row.set("trace_s", rr.seconds, 4);
					row.set("trace_s_cv", rr.cv(), 4);
					row.set("mrays_s", rr.mrays_per_second(), 4);
					// Only the timings come from the CPU kernel; every structural
					// column above is derived analytically.
					row.set("source_analytic", "S-analytic");
					row.set("source_timing", "S-cpu");
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

