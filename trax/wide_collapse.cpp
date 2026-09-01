#include "wide_collapse.h"

#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <build/collapse_weights.h>
#include <core/mode.h>
#include <core/traverse_bvh2.h>
#include <eval/direction_stats.h>
#include <eval/rayset.h>
#include <eval/trace.h>
#include <util/camera.h>
#include <util/log.h>
#include <util/mesh.h>
#include <util/metrics.h>
#include <util/timer.h>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace bvh;

namespace
{
	constexpr u32 collapse_width = 8u;
	const double  lambdas[] = { 0.25, 0.5, 0.75, 1.0 };
	const char*   view_names[] = { "A", "B", "C" };

	struct frozen_camera
	{
		std::string scene;
		std::string view;
		vec3 position, target, up;
		f32  focal{ 50.0f };
		bool found{ false };
	};

	std::vector<frozen_camera> load_cameras(const std::string& path)
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

		std::vector<std::string> cols;
		{
			std::stringstream ss(header);
			std::string c;
			while (std::getline(ss, c, ',')) { if (!c.empty() && c.back() == '\r') c.pop_back(); cols.push_back(c); }
		}
		auto index_of = [&](const char* name) -> int {
			for (size_t i = 0; i < cols.size(); ++i) if (cols[i] == name) return int(i);
			return -1;
			};

		const int i_scene = index_of("scene"), i_view = index_of("view");
		const int i_px = index_of("pos_x"), i_py = index_of("pos_y"), i_pz = index_of("pos_z");
		const int i_tx = index_of("target_x"), i_ty = index_of("target_y"), i_tz = index_of("target_z");
		const int i_ux = index_of("up_x"), i_uy = index_of("up_y"), i_uz = index_of("up_z");
		const int i_focal = index_of("focal");

		if (i_scene < 0 || i_view < 0 || i_px < 0 || i_tx < 0 || i_ux < 0 || i_focal < 0)
		{
			LOG_ERROR("frozen camera file '%s' is missing required columns", path.c_str());
			return out;
		}

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

			frozen_camera fc;
			fc.scene = v[i_scene];
			fc.view = v[i_view];
			fc.position = vec3(f32(std::atof(v[i_px].c_str())), f32(std::atof(v[i_py].c_str())), f32(std::atof(v[i_pz].c_str())));
			fc.target = vec3(f32(std::atof(v[i_tx].c_str())), f32(std::atof(v[i_ty].c_str())), f32(std::atof(v[i_tz].c_str())));
			fc.up = vec3(f32(std::atof(v[i_ux].c_str())), f32(std::atof(v[i_uy].c_str())), f32(std::atof(v[i_uz].c_str())));
			fc.focal = f32(std::atof(v[i_focal].c_str()));
			fc.found = true;
			out.push_back(fc);
		}
		return out;
	}

	// Per-ray closest-hit reference from the binary tree.
	struct ray_result
	{
		f32 t;
		u32 id;
	};

	std::vector<ray_result> trace_reference(const bvh2& tree, const mesh& m, const rayset& rs)
	{
		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		std::vector<ray_result> out(rs.size());
		for (u32 i = 0; i < rs.size(); ++i)
		{
			const ray r = rs.get(i);
			hit h;
			h.t = r.t_max;
			null_stats s;
			intersect(view, r, h, prims, s);
			out[i].t = h.valid() ? h.t : 0.0f;
			out[i].id = h.id;
		}
		return out;
	}

	struct equality_result
	{
		u64  checked{ 0 };
		u64  mismatches{ 0 };   // hit/miss or closest-t disagreement
		u64  tie_ids{ 0 };      // identical t, different primitive id
		bool pass{ false };
	};

	// Every measured ray must agree with the binary baseline on hit/miss and on
	// the closest-hit distance. A differing primitive id at an identical distance
	// is a legitimate tie and is counted, not failed.
	equality_result verify_against_reference(const bvh2& tree, const mesh& m, const rayset& rs,
		const std::vector<ray_result>& reference)
	{
		equality_result e;

		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		for (u32 i = 0; i < rs.size(); ++i)
		{
			const ray r = rs.get(i);
			hit h;
			h.t = r.t_max;
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

	double pct_change(double variant, double baseline)
	{
		return baseline > 0.0 ? 100.0 * (variant - baseline) / baseline : 0.0;
	}

} // namespace

bool run_wide_collapse(const wide_collapse_args& args)
{
	LOG_INFO("=== width-8 directional collapse: %s ===", args.scene_name.c_str());

	std::filesystem::create_directories(args.run_dir);
	const std::string csv = args.run_dir + "/w8_directional.csv";

	const std::vector<frozen_camera> all_cameras = load_cameras(args.cameras_csv);
	if (all_cameras.empty()) return false;

	std::vector<frozen_camera> views;
	for (const char* name : view_names)
	{
		for (const frozen_camera& fc : all_cameras)
			if (fc.scene == args.scene_name && fc.view == name) { views.push_back(fc); break; }
	}
	if (views.size() != 3)
	{
		LOG_ERROR("  expected 3 frozen views for '%s', found %zu",
			args.scene_name.c_str(), views.size());
		return false;
	}

	// ------------------------------------------------ one shared binary SAH tree
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

	// ------------------------------------------------- one rayset per frozen view
	rayset_args ra;
	ra.width = args.width;
	ra.height = args.height;

	std::vector<rayset>                   raysets(3);
	std::vector<std::vector<ray_result>>  references(3);
	std::vector<u64>                      hashes(3, 0);

	for (u32 v = 0; v < 3; ++v)
	{
		const camera cam(args.width, args.height, views[v].focal,
			views[v].position, views[v].target, views[v].up);

		raysets[v] = rayset::generate(ray_distribution::primary, m, binary, cam, ra);
		raysets[v].scene = args.scene_name;
		if (raysets[v].empty())
		{
			LOG_ERROR("  view %s produced no rays", views[v].view.c_str());
			return false;
		}
		hashes[v] = raysets[v].hash();
		references[v] = trace_reference(binary, m, raysets[v]);
	}

	// W from view A primary rays.
	const direction_weights W = compute_direction_weights(raysets[0]);
	if (!W.valid)
	{
		LOG_ERROR("  direction weights from view A are invalid");
		return false;
	}
	LOG_INFO("  W (view A) = (%.5f, %.5f, %.5f) over %llu rays, %llu rejected",
		W.wx, W.wy, W.wz, (unsigned long long)W.rays, (unsigned long long)W.rejected);

	// --------------------------------------------------------- baseline collapse
	std::vector<u8> emitted_baseline;
	trace_result    baseline_trace[3];
	u32             baseline_nodes = 0, baseline_depth = 0;

	{
		bvh2 wide = binary;

		collapse_args ca;
		ca.width = collapse_width;
		ca.method = collapse_method::dynamic_programming;
		ca.max_leaf_size = 1;
		ca.silent = true;

		const collapse_report cr = collapse(wide, m, ca, &emitted_baseline);
		if (!cr.stack_bound_ok)
		{
			LOG_ERROR("  baseline w8 needs a stack of %u (limit %u)", cr.required_stack, bvh2_stack_size);
			return false;
		}
		baseline_nodes = cr.node_count;
		baseline_depth = cr.max_depth;

		for (u32 v = 0; v < 3; ++v)
		{
			const equality_result eq = verify_against_reference(wide, m, raysets[v], references[v]);
			if (!eq.pass)
			{
				LOG_ERROR("  baseline w8 view %s: %llu/%llu rays disagree with the binary tree",
					views[v].view.c_str(), (unsigned long long)eq.mismatches,
					(unsigned long long)eq.checked);
				return false;
			}
			baseline_trace[v] = trace_rayset(wide, m, raysets[v], args.threads, 1u);

			metrics row;
			row.set("run_id", args.run_id);
			row.set("git_commit", args.git_commit);
			row.set("dirty", args.dirty ? 1u : 0u);
			row.set("scene", args.scene_name);
			row.set("triangles", m.triangle_count());
			row.set("view", views[v].view);
			row.set("matched_workload", v == 0 ? 1u : 0u);
			row.set("rayset_hash", std::to_string(hashes[v]));
			row.set("rays", i64(baseline_trace[v].rays));
			row.set("variant", "baseline_sah");
			row.set("lambda", 0.0, 4);
			row.set("width", collapse_width);
			row.set("wide_nodes", baseline_nodes);
			row.set("wide_depth", baseline_depth);
			row.set("changed_decisions", 0u);
			row.set("changed_decisions_pct", 0.0, 4);
			row.set("node_steps_per_ray", baseline_trace[v].node_steps_per_ray(), 4);
			row.set("box_tests_per_ray", baseline_trace[v].rays
				? double(baseline_trace[v].box_tests) / double(baseline_trace[v].rays) : 0.0, 4);
			row.set("tri_tests_per_ray", baseline_trace[v].tri_tests_per_ray(), 4);
			row.set("node_steps_pct", 0.0, 3);
			row.set("box_tests_pct", 0.0, 3);
			row.set("tri_tests_pct", 0.0, 3);
			row.set("hits", i64(baseline_trace[v].hits));
			row.set("correctness", "pass");
			row.set("mismatches", i64(eq.mismatches));
			row.set("tie_ids", i64(eq.tie_ids));
			row.set("Wx", W.wx, 6);
			row.set("Wy", W.wy, 6);
			row.set("Wz", W.wz, 6);
			row.set("binary_nodes", binary_nodes);
			row.set("binary_depth", binary.report().max_depth);
			row.set("source", "S-measured");
			if (!row.flush(csv)) return false;
		}

		LOG_INFO("  baseline w8: %u nodes, depth %u, view A %.3f node steps/ray",
			baseline_nodes, baseline_depth, baseline_trace[0].node_steps_per_ray());
	}

	// lambda = 0 through the directional weight path must reproduce the baseline
	// byte-for-byte. The weights are produced by calling surface_area() itself, so
	// the DP sees numerically identical inputs; if this ever fails, the weight
	// plumbing has perturbed the collapse and no comparison below is trustworthy.
	{
		const std::vector<f32> w0 = compute_collapse_weights(binary, 0.0, W);

		bvh2 check = binary;

		collapse_args ca;
		ca.width = collapse_width;
		ca.method = collapse_method::dynamic_programming;
		ca.max_leaf_size = 1;
		ca.silent = true;
		ca.node_weight = w0.data();
		ca.node_weight_count = static_cast<u32>(w0.size());

		std::vector<u8> emitted_zero;
		const collapse_report cr0 = collapse(check, m, ca, &emitted_zero);

		bool identical = cr0.node_count == baseline_nodes
			&& cr0.max_depth == baseline_depth
			&& emitted_zero.size() == emitted_baseline.size();
		if (identical)
			identical = std::memcmp(emitted_zero.data(), emitted_baseline.data(),
				emitted_zero.size()) == 0;

		if (!identical)
		{
			LOG_ERROR("  lambda=0 through the weight path did NOT reproduce the baseline collapse");
			return false;
		}
		LOG_INFO("  lambda=0 identity check: baseline reproduced exactly");
	}

	// ------------------------------------------------------ directional variants
	bool all_ok = true;

	for (double lambda : lambdas)
	{
		const std::vector<f32> weights = compute_collapse_weights(binary, lambda, W);

		bvh2 wide = binary;

		collapse_args ca;
		ca.width = collapse_width;
		ca.method = collapse_method::dynamic_programming;
		ca.max_leaf_size = 1;
		ca.silent = true;
		ca.node_weight = weights.data();
		ca.node_weight_count = static_cast<u32>(weights.size());

		std::vector<u8> emitted_dir;
		const collapse_report cr = collapse(wide, m, ca, &emitted_dir);

		if (!cr.stack_bound_ok)
		{
			LOG_ERROR("  lambda %.2f w8 needs a stack of %u (limit %u)",
				lambda, cr.required_stack, bvh2_stack_size);
			all_ok = false;
			continue;
		}

		double node_pct[3] = { 0.0, 0.0, 0.0 };

		u64 changed = 0;
		for (size_t i = 0; i < emitted_baseline.size(); ++i)
			if (emitted_baseline[i] != emitted_dir[i]) ++changed;
		const double changed_pct = binary_nodes ? 100.0 * double(changed) / double(binary_nodes) : 0.0;

		for (u32 v = 0; v < 3; ++v)
		{
			const equality_result eq = verify_against_reference(wide, m, raysets[v], references[v]);
			if (!eq.pass)
			{
				LOG_ERROR("  lambda %.2f view %s: %llu/%llu rays disagree with the binary tree",
					lambda, views[v].view.c_str(), (unsigned long long)eq.mismatches,
					(unsigned long long)eq.checked);
				all_ok = false;
				continue; // a failed variant publishes no row
			}

			const trace_result tr = trace_rayset(wide, m, raysets[v], args.threads, 1u);

			const double box_per_ray = tr.rays ? double(tr.box_tests) / double(tr.rays) : 0.0;
			const double base_box_per_ray = baseline_trace[v].rays
				? double(baseline_trace[v].box_tests) / double(baseline_trace[v].rays) : 0.0;

			metrics row;
			row.set("run_id", args.run_id);
			row.set("git_commit", args.git_commit);
			row.set("dirty", args.dirty ? 1u : 0u);
			row.set("scene", args.scene_name);
			row.set("triangles", m.triangle_count());
			row.set("view", views[v].view);
			row.set("matched_workload", v == 0 ? 1u : 0u);
			row.set("rayset_hash", std::to_string(hashes[v]));
			row.set("rays", i64(tr.rays));
			row.set("variant", "directional");
			row.set("lambda", lambda, 4);
			row.set("width", collapse_width);
			row.set("wide_nodes", cr.node_count);
			row.set("wide_depth", cr.max_depth);
			row.set("changed_decisions", i64(changed));
			row.set("changed_decisions_pct", changed_pct, 4);
			row.set("node_steps_per_ray", tr.node_steps_per_ray(), 4);
			row.set("box_tests_per_ray", box_per_ray, 4);
			row.set("tri_tests_per_ray", tr.tri_tests_per_ray(), 4);
			node_pct[v] = pct_change(tr.node_steps_per_ray(), baseline_trace[v].node_steps_per_ray());
			row.set("node_steps_pct", node_pct[v], 3);
			row.set("box_tests_pct", pct_change(box_per_ray, base_box_per_ray), 3);
			row.set("tri_tests_pct", pct_change(tr.tri_tests_per_ray(),
				baseline_trace[v].tri_tests_per_ray()), 3);
			row.set("hits", i64(tr.hits));
			row.set("correctness", "pass");
			row.set("mismatches", i64(eq.mismatches));
			row.set("tie_ids", i64(eq.tie_ids));
			row.set("Wx", W.wx, 6);
			row.set("Wy", W.wy, 6);
			row.set("Wz", W.wz, 6);
			row.set("binary_nodes", binary_nodes);
			row.set("binary_depth", binary.report().max_depth);
			row.set("source", "S-measured");
			if (!row.flush(csv)) all_ok = false;
		}

		LOG_INFO("  lambda %.2f: %u nodes (%+.2f%%), depth %u, decisions changed %.2f%%, "
			"view A node steps %+.2f%%, B %+.2f%%, C %+.2f%%",
			lambda, cr.node_count,
			100.0 * (double(cr.node_count) - double(baseline_nodes)) / double(baseline_nodes),
			cr.max_depth, changed_pct, node_pct[0], node_pct[1], node_pct[2]);
	}

	return all_ok;
}
