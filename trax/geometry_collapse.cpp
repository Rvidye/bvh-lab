#include "geometry_collapse.h"

#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <build/geometry_loss.h>
#include <core/mode.h>
#include <core/traverse_bvh2.h>
#include <eval/quality.h>
#include <eval/rayset.h>
#include <eval/trace.h>
#include <util/camera.h>
#include <util/image.h>
#include <util/log.h>
#include <util/mesh.h>
#include <util/metrics.h>
#include <util/timer.h>

#include <cmath>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace bvh;

namespace
{
	constexpr u32 collapse_width = 8u;

	// mu = 1.0 is the preregistered primary comparison; the rest are sensitivity.
	const double mus[] = { 0.25, 0.5, 1.0, 2.0 };
	constexpr double primary_mu = 1.0;

	const char* view_names[] = { "A", "B", "C" };

	struct frozen_camera
	{
		std::string view;
		vec3 position, target, up;
		f32  focal{ 50.0f };
	};

	std::vector<frozen_camera> load_cameras(const std::string& path, const std::string& scene)
	{
		std::vector<frozen_camera> out;

		std::ifstream f(path);
		if (!f.is_open())
		{
			LOG_ERROR("cannot open frozen camera file '%s'", path.c_str());
			return out;
		}

		std::string header;
		if (!std::getline(f, header)) return out;
		if (!header.empty() && header.back() == '\r') header.pop_back();

		std::vector<std::string> cols;
		{
			std::stringstream ss(header);
			std::string c;
			while (std::getline(ss, c, ',')) cols.push_back(c);
		}
		auto idx = [&](const char* n) -> int {
			for (size_t i = 0; i < cols.size(); ++i) if (cols[i] == n) return int(i);
			return -1;
			};

		const int i_scene = idx("scene"), i_view = idx("view");
		const int i_px = idx("pos_x"), i_py = idx("pos_y"), i_pz = idx("pos_z");
		const int i_tx = idx("target_x"), i_ty = idx("target_y"), i_tz = idx("target_z");
		const int i_ux = idx("up_x"), i_uy = idx("up_y"), i_uz = idx("up_z");
		const int i_focal = idx("focal");
		if (i_scene < 0 || i_view < 0 || i_px < 0 || i_tx < 0 || i_ux < 0 || i_focal < 0) return out;

		std::vector<frozen_camera> found;
		std::string line;
		while (std::getline(f, line))
		{
			if (!line.empty() && line.back() == '\r') line.pop_back();
			if (line.empty()) continue;

			std::vector<std::string> v;
			std::stringstream ss(line);
			std::string c;
			while (std::getline(ss, c, ',')) v.push_back(c);
			if (v.size() < cols.size()) continue;
			if (v[i_scene] != scene) continue;

			frozen_camera fc;
			fc.view = v[i_view];
			fc.position = vec3(f32(std::atof(v[i_px].c_str())), f32(std::atof(v[i_py].c_str())), f32(std::atof(v[i_pz].c_str())));
			fc.target = vec3(f32(std::atof(v[i_tx].c_str())), f32(std::atof(v[i_ty].c_str())), f32(std::atof(v[i_tz].c_str())));
			fc.up = vec3(f32(std::atof(v[i_ux].c_str())), f32(std::atof(v[i_uy].c_str())), f32(std::atof(v[i_uz].c_str())));
			fc.focal = f32(std::atof(v[i_focal].c_str()));
			found.push_back(fc);
		}

		for (const char* name : view_names)
			for (const frozen_camera& fc : found)
				if (fc.view == name) { out.push_back(fc); break; }
		return out;
	}

	struct ray_result { f32 t; u32 id; };

	std::vector<ray_result> trace_reference(const bvh2& tree, const mesh& m, const rayset& rs)
	{
		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		std::vector<ray_result> out(rs.size());
		for (u32 i = 0; i < rs.size(); ++i)
		{
			const ray r = rs.get(i);
			hit h; h.t = r.t_max;
			null_stats s;
			intersect(view, r, h, prims, s);
			out[i].t = h.valid() ? h.t : 0.0f;
			out[i].id = h.id;
		}
		return out;
	}

	struct equality_result
	{
		u64 checked{ 0 }, mismatches{ 0 }, tie_ids{ 0 };
		bool pass{ false };
	};

	equality_result verify(const bvh2& tree, const mesh& m, const rayset& rs,
		const std::vector<ray_result>& reference)
	{
		equality_result e;

		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		for (u32 i = 0; i < rs.size(); ++i)
		{
			const ray r = rs.get(i);
			hit h; h.t = r.t_max;
			null_stats s;
			intersect(view, r, h, prims, s);

			++e.checked;
			const bool ref_hit = reference[i].id != invalid_id;
			if (h.valid() != ref_hit) { ++e.mismatches; continue; }
			if (!ref_hit) continue;
			if (h.t != reference[i].t) { ++e.mismatches; continue; }
			if (h.id != reference[i].id) ++e.tie_ids;
		}
		e.pass = (e.mismatches == 0);
		return e;
	}

	double pct(double v, double base) { return base > 0.0 ? 100.0 * (v - base) / base : 0.0; }

	struct variant
	{
		std::string       label;
		collapse_loss     kind{ collapse_loss::none };
		double            mu{ 0.0 };
		bvh2              tree;
		std::vector<u8>   emitted;
		collapse_report   report;
		quality_metrics   quality;
		loss_totals       loss;
		u64               changed{ 0 };
	};

} // namespace

bool run_geometry_collapse(const geometry_collapse_args& args)
{
	LOG_INFO("=== geometry-derived directional BVH8 collapse: %s ===", args.scene_name.c_str());

	std::filesystem::create_directories(args.run_dir);
	std::filesystem::create_directories(args.image_dir);
	const std::string csv = args.run_dir + "/san_miguel_geometry_collapse.csv";

	const std::vector<frozen_camera> views = load_cameras(args.cameras_csv, args.scene_name);
	if (views.size() != 3)
	{
		LOG_ERROR("  expected 3 frozen views for '%s', found %zu", args.scene_name.c_str(), views.size());
		return false;
	}

	// ---------------------------------------------------- shared binary SAH tree
	mesh m;
	if (!m.load_obj(args.scene_path))
	{
		LOG_ERROR("  could not load '%s'", args.scene_path.c_str());
		return false;
	}

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.bins = args.bins;
	ba.max_leaf_size = 1;
	ba.silent = true;

	bvh2 binary;
	timer build_timer;
	binary.build(m, ba);
	binary.apply_reorder(m);
	binary.refit(m);

	const u32 binary_nodes = binary.report().node_count;
	LOG_INFO("  binary SAH: %u nodes, depth %u (%.1f s)",
		binary_nodes, binary.report().max_depth, build_timer.elapsed_ms() / 1000.0);

	// =========================================================================
	// EVERY TREE IS BUILT HERE, BEFORE A SINGLE EVALUATION RAY EXISTS.
	// No camera, rayset, view or workload is in scope during collapse.
	// =========================================================================

	std::vector<variant> variants;

	auto build_variant = [&](const std::string& label, collapse_loss kind, double mu) -> bool
		{
			variant v;
			v.label = label;
			v.kind = kind;
			v.mu = mu;

			const geometry_loss_args gla{ kind, mu };
			const std::vector<f32> internal_area = compute_internal_cost_area(binary, m, gla);

			v.tree = binary;

			collapse_args ca;
			ca.width = collapse_width;
			ca.method = collapse_method::dynamic_programming;
			ca.max_leaf_size = 1;
			ca.silent = true;
			if (kind != collapse_loss::none && mu != 0.0)
			{
				ca.node_internal_area = internal_area.data();
				ca.node_internal_area_count = static_cast<u32>(internal_area.size());
			}

			v.report = collapse(v.tree, m, ca, &v.emitted);

			// The Phase-0 safety bound is mandatory and must never be bypassed.
			if (!v.report.stack_bound_ok)
			{
				LOG_ERROR("  %s needs a traversal stack of %u (limit %u)",
					label.c_str(), v.report.required_stack, bvh2_stack_size);
				return false;
			}

			quality_args qa;
			qa.compute_epo = false;
			v.quality = evaluate(v.tree, m, qa);
			v.loss = sum_internal_loss(v.tree, m);

			LOG_INFO("  built %-22s nodes %u depth %u fullness %.2f sah %.4f Ldir %.4f Lscalar %.4f",
				label.c_str(), v.report.node_count, v.report.max_depth, v.report.mean_fullness,
				v.quality.sah_cost, v.loss.directional, v.loss.scalar_density);

			variants.push_back(std::move(v));
			return true;
		};

	if (!build_variant("sah_baseline", collapse_loss::none, 0.0)) return false;

	// mu = 0 through both loss paths must reproduce the ordinary collapse exactly.
	for (collapse_loss kind : { collapse_loss::directional, collapse_loss::scalar_density })
	{
		const geometry_loss_args gla{ kind, 0.0 };
		const std::vector<f32> zero_area = compute_internal_cost_area(binary, m, gla);

		bvh2 check = binary;

		collapse_args ca;
		ca.width = collapse_width;
		ca.method = collapse_method::dynamic_programming;
		ca.max_leaf_size = 1;
		ca.silent = true;
		ca.node_internal_area = zero_area.data();
		ca.node_internal_area_count = static_cast<u32>(zero_area.size());

		std::vector<u8> emitted_zero;
		const collapse_report cr = collapse(check, m, ca, &emitted_zero);

		const bool identical =
			cr.node_count == variants[0].report.node_count &&
			cr.max_depth == variants[0].report.max_depth &&
			check.nodes().size() == variants[0].tree.nodes().size() &&
			std::memcmp(check.nodes().data(), variants[0].tree.nodes().data(),
				check.nodes().size() * sizeof(bvh2_node)) == 0 &&
			check.prim_indices() == variants[0].tree.prim_indices() &&
			emitted_zero.size() == variants[0].emitted.size() &&
			std::memcmp(emitted_zero.data(), variants[0].emitted.data(), emitted_zero.size()) == 0;

		if (!identical)
		{
			LOG_ERROR("  mu=0 via %s did NOT reproduce the ordinary collapse byte-for-byte",
				to_string(kind));
			return false;
		}
		LOG_INFO("  mu=0 via %-12s reproduces the ordinary collapse byte-for-byte", to_string(kind));
	}

	for (double mu : mus)
	{
		char label[64];
		snprintf(label, sizeof(label), "scalar_mu%.2f", mu);
		if (!build_variant(label, collapse_loss::scalar_density, mu)) return false;
	}
	for (double mu : mus)
	{
		char label[64];
		snprintf(label, sizeof(label), "directional_mu%.2f", mu);
		if (!build_variant(label, collapse_loss::directional, mu)) return false;
	}

	// Retained/absorbed decisions changed versus the baseline.
	for (variant& v : variants)
	{
		u64 changed = 0;
		for (size_t i = 0; i < v.emitted.size(); ++i)
			if (v.emitted[i] != variants[0].emitted[i]) ++changed;
		v.changed = changed;
	}

	// =========================================================================
	// Only now do evaluation rays come into existence.
	// =========================================================================

	rayset_args ra;
	ra.width = args.width;
	ra.height = args.height;

	std::vector<rayset>                  raysets(3);
	std::vector<std::vector<ray_result>> references(3);
	std::vector<u64>                     hashes(3, 0);
	std::vector<camera>                  cams(3);

	for (u32 v = 0; v < 3; ++v)
	{
		cams[v] = camera(args.width, args.height, views[v].focal,
			views[v].position, views[v].target, views[v].up);
		raysets[v] = rayset::generate(ray_distribution::primary, m, binary, cams[v], ra);
		raysets[v].scene = args.scene_name;
		if (raysets[v].empty())
		{
			LOG_ERROR("  view %s produced no rays", views[v].view.c_str());
			return false;
		}
		hashes[v] = raysets[v].hash();
		references[v] = trace_reference(binary, m, raysets[v]);
	}

	bool all_ok = true;
	const variant& base = variants[0];
	std::vector<trace_result> base_trace(3);

	for (u32 vi = 0; vi < 3; ++vi)
		base_trace[vi] = trace_rayset(base.tree, m, raysets[vi], args.threads, 1u);

	for (const variant& v : variants)
	{
		for (u32 vi = 0; vi < 3; ++vi)
		{
			const equality_result eq = verify(v.tree, m, raysets[vi], references[vi]);
			if (!eq.pass)
			{
				LOG_ERROR("  %s view %s: %llu/%llu rays disagree with the binary tree",
					v.label.c_str(), views[vi].view.c_str(),
					(unsigned long long)eq.mismatches, (unsigned long long)eq.checked);
				all_ok = false;
				continue;
			}

			const trace_result tr = trace_rayset(v.tree, m, raysets[vi], args.threads, 1u);
			const double rays = double(tr.rays);
			const double b_rays = double(base_trace[vi].rays);

			auto per_ray = [](u64 n, double r) { return r > 0.0 ? double(n) / r : 0.0; };

			metrics row;
			row.set("run_id", args.run_id);
			row.set("git_commit", args.git_commit);
			row.set("dirty", args.dirty ? 1u : 0u);
			row.set("scene", args.scene_name);
			row.set("triangles", m.triangle_count());
			row.set("view", views[vi].view);
			row.set("rayset_hash", std::to_string(hashes[vi]));
			row.set("rays", i64(tr.rays));
			row.set("variant", v.label);
			row.set("loss_kind", to_string(v.kind));
			row.set("mu", v.mu, 4);
			row.set("is_primary_comparison",
				(v.kind == collapse_loss::directional && v.mu == primary_mu) ? 1u : 0u);
			row.set("width", collapse_width);

			row.set("wide_nodes", v.report.node_count);
			row.set("emitted_depth", v.report.max_depth);
			row.set("mean_fullness", v.report.mean_fullness, 4);
			row.set("decisions_changed", i64(v.changed));
			row.set("decisions_changed_pct",
				binary_nodes ? 100.0 * double(v.changed) / double(binary_nodes) : 0.0, 4);
			row.set("sah_cost", v.quality.sah_cost, 6);
			row.set("dir_loss_cost", v.loss.directional, 6);
			row.set("scalar_loss_cost", v.loss.scalar_density, 6);
			row.set("bvh_bytes", i64(u64(v.report.node_count) * sizeof(bvh2_node)));

			row.set("node_steps_per_ray", per_ray(tr.node_steps, rays), 4);
			row.set("box_tests_per_ray", per_ray(tr.box_tests, rays), 4);
			row.set("prim_steps_per_ray", per_ray(tr.prim_steps, rays), 4);
			row.set("tri_tests_per_ray", per_ray(tr.tri_tests, rays), 4);
			row.set("box_hits_per_ray", per_ray(tr.box_hits, rays), 4);
			row.set("pruned_pops_per_ray", per_ray(tr.pruned_pops, rays), 4);
			row.set("max_stack", tr.max_stack);

			row.set("node_steps_pct", pct(per_ray(tr.node_steps, rays), per_ray(base_trace[vi].node_steps, b_rays)), 3);
			row.set("box_tests_pct", pct(per_ray(tr.box_tests, rays), per_ray(base_trace[vi].box_tests, b_rays)), 3);
			row.set("prim_steps_pct", pct(per_ray(tr.prim_steps, rays), per_ray(base_trace[vi].prim_steps, b_rays)), 3);
			row.set("tri_tests_pct", pct(per_ray(tr.tri_tests, rays), per_ray(base_trace[vi].tri_tests, b_rays)), 3);
			row.set("box_hits_pct", pct(per_ray(tr.box_hits, rays), per_ray(base_trace[vi].box_hits, b_rays)), 3);
			row.set("pruned_pops_pct", pct(per_ray(tr.pruned_pops, rays), per_ray(base_trace[vi].pruned_pops, b_rays)), 3);
			row.set("wide_nodes_pct", pct(double(v.report.node_count), double(base.report.node_count)), 3);

			row.set("hits", i64(tr.hits));
			row.set("correctness", "pass");
			row.set("mismatches", i64(eq.mismatches));
			row.set("tie_ids", i64(eq.tie_ids));
			row.set("rays_used_in_construction", 0u);
			row.set("source", "S-measured");
			if (!row.flush(csv)) all_ok = false;
		}

		LOG_INFO("  %-22s decisions changed %.2f%%  node steps A %+.2f%% B %+.2f%% C %+.2f%%",
			v.label.c_str(),
			binary_nodes ? 100.0 * double(v.changed) / double(binary_nodes) : 0.0,
			pct(double(trace_rayset(v.tree, m, raysets[0], args.threads, 1u).node_steps),
				double(base_trace[0].node_steps)),
			pct(double(trace_rayset(v.tree, m, raysets[1], args.threads, 1u).node_steps),
				double(base_trace[1].node_steps)),
			pct(double(trace_rayset(v.tree, m, raysets[2], args.threads, 1u).node_steps),
				double(base_trace[2].node_steps)));
	}

	// ------------------------------------------------- native traversal heatmaps
	{
		const variant* primary = nullptr;
		for (const variant& v : variants)
			if (v.kind == collapse_loss::directional && v.mu == primary_mu) primary = &v;

		if (!primary)
		{
			LOG_ERROR("  no directional mu=%.2f variant to render", primary_mu);
			return false;
		}

		image img_base(args.width, args.height), img_dir(args.width, args.height);
		std::vector<u32> counts_base, counts_dir;

		render_bvh2(base.tree, m, cams[0], img_base, &counts_base, args.threads);
		render_bvh2(primary->tree, m, cams[0], img_dir, &counts_dir, args.threads);

		// One shared colour scale from the combined arrays, passed explicitly to
		// both, so the two images are directly comparable.
		u32 shared_max = 0;
		for (u32 c : counts_base) shared_max = bvh::max(shared_max, c);
		for (u32 c : counts_dir)  shared_max = bvh::max(shared_max, c);

		LOG_INFO("  shared heatmap colour scale (max traversal steps per pixel): %u", shared_max);

		const std::string p_base = args.image_dir + "/san-miguel_view-A_w8-sah_traversal-heatmap.png";
		const std::string p_dir = args.image_dir + "/san-miguel_view-A_w8-direction-D-mu1_traversal-heatmap.png";
		const std::string p_col = args.image_dir + "/san-miguel_view-A_color.png";

		const bool ok =
			image::from_counts(counts_base, args.width, args.height, shared_max).write_png(p_base, false) &&
			image::from_counts(counts_dir, args.width, args.height, shared_max).write_png(p_dir, false) &&
			img_base.write_png(p_col);

		if (!ok) { LOG_ERROR("  heatmap write failed"); all_ok = false; }

		metrics row;
		row.set("run_id", args.run_id);
		row.set("git_commit", args.git_commit);
		row.set("dirty", args.dirty ? 1u : 0u);
		row.set("scene", args.scene_name);
		row.set("view", "A");
		row.set("heatmap_shared_max_steps", shared_max);
		row.set("heatmap_baseline", p_base);
		row.set("heatmap_directional", p_dir);
		row.set("color_render", p_col);
		row.set("heatmap_metric", "trace_stats::total_steps (node_steps + prim_steps) per pixel");
		row.flush(args.run_dir + "/san_miguel_heatmaps.csv");
	}

	return all_ok;
}

// ------------------------------------------------------------ node-term dump
//
// WP-C: the per-node data that makes a collapse run analysable. Without it the
// report can say WHAT changed but not WHY.
//
// A full per-node CSV would be 19.9M rows for San Miguel, which is neither
// committable nor readable. The questions the plan actually asks -- per-axis
// saturation rate, whether the three fills are correlated or anticorrelated,
// the distribution of q -- are aggregate questions, so they are answered over
// EVERY node in d2_node_summary.csv, and d2_node_terms.csv carries a
// deterministic stride sample for inspection.

namespace
{
	constexpr u32 node_term_sample_cap = 200000u;

	// q histogram: 500 bins over [0,5) plus an overflow bucket, which gives
	// quantiles to about 0.01 with O(1) memory.
	constexpr u32 q_bins = 500u;
	constexpr double q_hist_max = 5.0;

	struct axis_stats
	{
		u64    valid{ 0 };
		u64    degenerate{ 0 };
		u64    saturated{ 0 };
		double q_sum{ 0.0 };
		double fill_sum{ 0.0 };
		u64    hist[q_bins + 1]{};

		void add(bool is_valid, double q, double fill)
		{
			if (!is_valid) { ++degenerate; return; }
			++valid;
			q_sum += q;
			fill_sum += fill;
			if (q >= 1.0) ++saturated;
			const u32 b = q >= q_hist_max
				? q_bins
				: static_cast<u32>(q / q_hist_max * double(q_bins));
			++hist[b < q_bins ? b : q_bins];
		}

		double quantile(double p) const
		{
			if (valid == 0) return 0.0;
			const u64 target = static_cast<u64>(p * double(valid));
			u64 seen = 0;
			for (u32 b = 0; b < q_bins; ++b)
			{
				seen += hist[b];
				if (seen >= target)
					return (double(b) + 0.5) * q_hist_max / double(q_bins);
			}
			return q_hist_max;
		}
	};

	// Running Pearson correlation between two fill series.
	struct correlation
	{
		u64    n{ 0 };
		double sx{ 0.0 }, sy{ 0.0 }, sxx{ 0.0 }, syy{ 0.0 }, sxy{ 0.0 };

		void add(double x, double y)
		{
			++n; sx += x; sy += y; sxx += x * x; syy += y * y; sxy += x * y;
		}

		double value() const
		{
			if (n < 2) return 0.0;
			const double cov = sxy / double(n) - (sx / double(n)) * (sy / double(n));
			const double vx = sxx / double(n) - (sx / double(n)) * (sx / double(n));
			const double vy = syy / double(n) - (sy / double(n)) * (sy / double(n));
			const double d = std::sqrt(vx * vy);
			return d > 0.0 ? cov / d : 0.0;
		}
	};

	std::vector<u32> node_depths(const bvh2& tree)
	{
		const std::vector<bvh2_node>& nodes = tree.nodes();
		std::vector<u32> depth(nodes.size(), 0u);
		for (size_t i = 0; i < nodes.size(); ++i)
		{
			if (!nodes[i].ptr.is_int) continue;
			for (u32 c = 0; c < nodes[i].ptr.child_cnt; ++c)
				depth[nodes[i].ptr.child_idx + c] = depth[i] + 1u;
		}
		return depth;
	}

} // namespace

bool run_node_terms(const geometry_collapse_args& args)
{
	LOG_INFO("=== node terms: %s ===", args.scene_name.c_str());

	std::filesystem::create_directories(args.run_dir);
	const std::string terms_csv = args.run_dir + "/d2_node_terms.csv";
	const std::string summary_csv = args.run_dir + "/d2_node_summary.csv";

	mesh m;
	if (!m.load_obj(args.scene_path))
	{
		LOG_ERROR("  could not load '%s'", args.scene_path.c_str());
		return false;
	}

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.bins = args.bins;
	ba.max_leaf_size = 1;
	ba.silent = true;

	bvh2 binary;
	binary.build(m, ba);
	binary.apply_reorder(m);
	binary.refit(m);

	const std::vector<bvh2_node>& nodes = binary.nodes();
	const std::vector<directional_geometry> g = compute_directional_geometry(binary, m);
	const std::vector<u32> depth = node_depths(binary);

	LOG_INFO("  %zu binary nodes, depth %u", nodes.size(), binary.report().max_depth);

	// Deterministic stride so the sample is reproducible and spread over the tree.
	const u64 stride = nodes.size() > node_term_sample_cap
		? (nodes.size() + node_term_sample_cap - 1) / node_term_sample_cap
		: 1u;

	std::ofstream out(terms_csv, std::ios::app);
	if (!out.is_open()) { LOG_ERROR("  cannot open %s", terms_csv.c_str()); return false; }
	if (std::filesystem::file_size(terms_csv) == 0)
		out << "run_id,scene,node_id,depth,is_internal,prim_count,subtree_prims,"
		"sa,fx,fy,fz,g_yz,g_xz,g_xy,tri_area_sum,"
		"q_x,q_y,q_z,fill_x,fill_y,fill_z,"
		"q_valid_x,q_valid_y,q_valid_z,q_saturated_x,q_saturated_y,q_saturated_z,"
		"l_dir,l_scalar,l_dir_min,l_dir_spread\n";

	axis_stats  ax[3];
	correlation corr_xy, corr_xz, corr_yz;
	u64 nodes_all = 0, nodes_internal = 0;
	u64 sat_count_hist[4]{};       // nodes with 0..3 saturated axes
	u64 any_degenerate = 0;
	double l_dir_over_sa = 0.0, l_min_over_sa = 0.0, l_scalar_over_sa = 0.0;
	u64 sa_positive = 0;

	char buf[768];

	for (size_t i = 0; i < nodes.size(); ++i)
	{
		const aabb& box = nodes[i].bounds;
		const directional_geometry& gi = g[i];

		const vec3 e = box.extent();
		const double fx = double(e.y) * double(e.z);
		const double fy = double(e.x) * double(e.z);
		const double fz = double(e.x) * double(e.y);
		const double sa = double(box.surface_area());

		const axis_fill a = compute_axis_fill(box, gi);

		const f32 l_dir = directional_loss(box, gi);
		const f32 l_scalar = scalar_density_loss(box, gi);
		const f32 l_min = directional_min_loss(box, gi);
		const f32 l_spread = directional_spread_loss(box, gi);

		++nodes_all;
		if (nodes[i].ptr.is_int) ++nodes_internal;

		u32 sat = 0;
		for (u32 k = 0; k < 3; ++k)
		{
			ax[k].add(a.valid[k], a.q[k], a.fill[k]);
			if (a.valid[k] && a.saturated[k]) ++sat;
		}
		++sat_count_hist[sat < 4 ? sat : 3];
		if (a.valid_count < 3) ++any_degenerate;

		// Correlations only over nodes where both axes are meaningful.
		if (a.valid[0] && a.valid[1]) corr_xy.add(a.fill[0], a.fill[1]);
		if (a.valid[0] && a.valid[2]) corr_xz.add(a.fill[0], a.fill[2]);
		if (a.valid[1] && a.valid[2]) corr_yz.add(a.fill[1], a.fill[2]);

		if (sa > 0.0)
		{
			++sa_positive;
			l_dir_over_sa += double(l_dir) / sa;
			l_min_over_sa += double(l_min) / sa;
			l_scalar_over_sa += double(l_scalar) / sa;
		}

		if (i % stride) continue;

		const int n = snprintf(buf, sizeof(buf),
			"%s,%s,%zu,%u,%u,%u,%u,"
			"%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
			"%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,"
			"%u,%u,%u,%u,%u,%u,"
			"%.6g,%.6g,%.6g,%.6g\n",
			args.run_id.c_str(), args.scene_name.c_str(), i, depth[i],
			nodes[i].ptr.is_int ? 1u : 0u,
			nodes[i].ptr.is_int ? 0u : u32(nodes[i].ptr.prim_cnt),
			gi.primitive_count,
			sa, fx, fy, fz, gi.p_yz, gi.p_xz, gi.p_xy, gi.triangle_surface_area_sum,
			a.q[0], a.q[1], a.q[2], a.fill[0], a.fill[1], a.fill[2],
			a.valid[0] ? 1u : 0u, a.valid[1] ? 1u : 0u, a.valid[2] ? 1u : 0u,
			a.saturated[0] ? 1u : 0u, a.saturated[1] ? 1u : 0u, a.saturated[2] ? 1u : 0u,
			double(l_dir), double(l_scalar), double(l_min), double(l_spread));
		out.write(buf, n);
	}

	out.flush();
	if (!out.good()) { LOG_ERROR("  write failed for %s", terms_csv.c_str()); return false; }

	const u64 valid_total = ax[0].valid + ax[1].valid + ax[2].valid;
	const u64 sat_total = ax[0].saturated + ax[1].saturated + ax[2].saturated;

	LOG_INFO("  saturation rate over valid axes: %.4f  (x %.4f, y %.4f, z %.4f)",
		valid_total ? double(sat_total) / double(valid_total) : 0.0,
		ax[0].valid ? double(ax[0].saturated) / double(ax[0].valid) : 0.0,
		ax[1].valid ? double(ax[1].saturated) / double(ax[1].valid) : 0.0,
		ax[2].valid ? double(ax[2].saturated) / double(ax[2].valid) : 0.0);
	LOG_INFO("  fill correlation: xy %.4f  xz %.4f  yz %.4f",
		corr_xy.value(), corr_xz.value(), corr_yz.value());
	LOG_INFO("  nodes with 0/1/2/3 saturated axes: %llu / %llu / %llu / %llu",
		(unsigned long long)sat_count_hist[0], (unsigned long long)sat_count_hist[1],
		(unsigned long long)sat_count_hist[2], (unsigned long long)sat_count_hist[3]);

	metrics row;
	row.set("run_id", args.run_id);
	row.set("git_commit", args.git_commit);
	row.set("dirty", args.dirty ? 1u : 0u);
	row.set("scene", args.scene_name);
	row.set("triangles", m.triangle_count());
	row.set("binary_nodes", i64(nodes_all));
	row.set("internal_nodes", i64(nodes_internal));
	row.set("binary_depth", binary.report().max_depth);
	row.set("sample_stride", i64(stride));
	row.set("sampled_rows", i64((nodes.size() + stride - 1) / stride));

	row.set("valid_axes", i64(valid_total));
	row.set("degenerate_axes", i64(ax[0].degenerate + ax[1].degenerate + ax[2].degenerate));
	row.set("nodes_with_degenerate_axis", i64(any_degenerate));
	row.set("saturation_rate_all_axes",
		valid_total ? double(sat_total) / double(valid_total) : 0.0, 6);
	for (u32 k = 0; k < 3; ++k)
	{
		const char* names[3] = { "x", "y", "z" };
		row.set((std::string("saturation_rate_") + names[k]).c_str(),
			ax[k].valid ? double(ax[k].saturated) / double(ax[k].valid) : 0.0, 6);
		row.set((std::string("q_mean_") + names[k]).c_str(),
			ax[k].valid ? ax[k].q_sum / double(ax[k].valid) : 0.0, 6);
		row.set((std::string("q_p10_") + names[k]).c_str(), ax[k].quantile(0.10), 4);
		row.set((std::string("q_p50_") + names[k]).c_str(), ax[k].quantile(0.50), 4);
		row.set((std::string("q_p90_") + names[k]).c_str(), ax[k].quantile(0.90), 4);
		row.set((std::string("fill_mean_") + names[k]).c_str(),
			ax[k].valid ? ax[k].fill_sum / double(ax[k].valid) : 0.0, 6);
	}
	row.set("nodes_0_saturated_axes", i64(sat_count_hist[0]));
	row.set("nodes_1_saturated_axes", i64(sat_count_hist[1]));
	row.set("nodes_2_saturated_axes", i64(sat_count_hist[2]));
	row.set("nodes_3_saturated_axes", i64(sat_count_hist[3]));
	row.set("fill_corr_xy", corr_xy.value(), 6);
	row.set("fill_corr_xz", corr_xz.value(), 6);
	row.set("fill_corr_yz", corr_yz.value(), 6);
	row.set("mean_l_dir_over_sa", sa_positive ? l_dir_over_sa / double(sa_positive) : 0.0, 6);
	row.set("mean_l_dir_min_over_sa", sa_positive ? l_min_over_sa / double(sa_positive) : 0.0, 6);
	row.set("mean_l_scalar_over_sa", sa_positive ? l_scalar_over_sa / double(sa_positive) : 0.0, 6);
	row.set("source", "S-measured");

	return row.flush(summary_csv);
}
