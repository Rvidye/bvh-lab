#include "phase0.h"

#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <core/mode.h>
#include <core/trace_stats.h>
#include <core/traverse_bvh2.h>
#include <eval/trace.h>
#include <reference/brute_force.h>
#include <util/camera.h>
#include <util/log.h>
#include <util/memory.h>
#include <util/mesh.h>
#include <util/metrics.h>
#include <util/parallel.h>
#include <util/timer.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace bvh;

namespace
{
	// Widths measured. 8 is the primary configuration; 4 and 16 are the
	// sensitivity checks of the plan.
	const u32 widths[] = { 4u, 8u, 16u };

	// Camera screening thresholds.
	//
	// The plan's original upper bound of 0.98 on hit fraction was written to
	// reject "a wall filling the frame". Measurement showed it rejects 178 of 192
	// candidates across the first three scenes because these are ENCLOSED
	// interiors, where a hit fraction of exactly 1.0 is the correct and desirable
	// condition rather than a defect. The upper bound is therefore removed and its
	// actual purpose -- rejecting a camera pressed against a surface -- is served
	// by a near-field test on the 10th percentile hit distance, which measures the
	// thing the upper bound was a poor proxy for. Recorded as a Phase-0 deviation;
	// no performance number existed when this was changed.
	constexpr double min_hit_fraction = 0.55;
	constexpr double min_median_hit_fraction_of_diagonal = 0.05;
	constexpr double min_p10_hit_fraction_of_diagonal = 0.005;
	constexpr double min_view_separation_degrees = 45.0;
	constexpr double min_eye_separation_fraction_of_diagonal = 0.05;

	double mb(u64 bytes) { return double(bytes) / (1024.0 * 1024.0); }

	struct candidate_camera
	{
		u32    index{ 0 };
		vec3   position{};
		vec3   target{};
		vec3   up{};
		f32    focal{ 50.0f };
		vec3   direction{};        // normalised target - position
		double hit_fraction{ 0.0 };
		double median_hit_over_diagonal{ 0.0 };
		double p10_hit_over_diagonal{ 0.0 };
		bool   accepted{ false };
		const char* reject_reason{ "" };
	};

	// A deterministic candidate: eye just off one triangle's surface on the side
	// facing the scene centre, looking at another triangle's centroid. Using real
	// geometry keeps interior scenes interior, which camera::frame_bounds does not.
	candidate_camera make_candidate(const mesh& m, u32 i, f32 diagonal, const vec3& scene_centre)
	{
		const u32 count = m.triangle_count();

		const u32 a = static_cast<u32>((u64(i) * 7919ull + 13ull) % count);
		const u32 b = static_cast<u32>((u64(i) * 104729ull + 12345ull) % count);

		const triangle ta = m.get_triangle(a);
		const triangle tb = m.get_triangle(b);

		vec3 n = ta.normal();
		const vec3 ca = ta.centroid();
		if (dot(n, scene_centre - ca) < 0.0f) n = -n;

		candidate_camera c;
		c.index = i;
		c.position = ca + n * (0.004f * diagonal);
		c.target = tb.centroid();

		const vec3 delta = c.target - c.position;
		const f32  len = length(delta);
		c.direction = len > 0.0f ? delta / len : vec3(0.0f, 0.0f, -1.0f);

		// Keep the up vector away from the view direction.
		c.up = (bvh::abs(c.direction.y) > 0.95f) ? vec3(0.0f, 0.0f, 1.0f) : vec3(0.0f, 1.0f, 0.0f);
		return c;
	}

	// Screens a candidate with a small primary trace and records coverage.
	void screen_candidate(candidate_camera& c, const bvh2& tree, const mesh& m,
		u32 res, f32 diagonal)
	{
		const f32 separation = length(c.target - c.position);
		if (separation < 0.02f * diagonal)
		{
			c.reject_reason = "eye and target too close";
			return;
		}

		const camera cam(res, res, c.focal, c.position, c.target, c.up);

		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		std::vector<f32> hits;
		hits.reserve(size_t(res) * res);

		u64 traced = 0;
		for (u32 j = 0; j < res; ++j)
		{
			for (u32 i = 0; i < res; ++i)
			{
				const ray r = cam.generate_ray_through_pixel(i, j);
				hit h;
				h.t = r.t_max;
				null_stats s;
				intersect(view, r, h, prims, s);
				++traced;
				if (h.valid()) hits.push_back(h.t);
			}
		}

		c.hit_fraction = traced ? double(hits.size()) / double(traced) : 0.0;

		if (!hits.empty())
		{
			std::sort(hits.begin(), hits.end());
			const f32 median = hits[hits.size() / 2];
			const f32 p10 = hits[hits.size() / 10];
			c.median_hit_over_diagonal = diagonal > 0.0f ? double(median) / double(diagonal) : 0.0;
			c.p10_hit_over_diagonal = diagonal > 0.0f ? double(p10) / double(diagonal) : 0.0;
		}

		if (c.hit_fraction < min_hit_fraction) { c.reject_reason = "hit fraction too low"; return; }
		if (c.median_hit_over_diagonal < min_median_hit_fraction_of_diagonal)
		{
			c.reject_reason = "median hit distance too short";
			return;
		}
		if (c.p10_hit_over_diagonal < min_p10_hit_fraction_of_diagonal)
		{
			c.reject_reason = "near-field: eye pressed against a surface";
			return;
		}

		c.accepted = true;
		c.reject_reason = "";
	}

	double angle_between_degrees(const vec3& a, const vec3& b)
	{
		const double d = double(dot(a, b));
		const double clamped = d < -1.0 ? -1.0 : (d > 1.0 ? 1.0 : d);
		return std::acos(clamped) * 180.0 / 3.14159265358979323846;
	}

	// Brute-force oracle throughput, in triangle tests per second.
	double oracle_throughput(const mesh& m, const camera& cam, u32 rays, u32 threads,
		double& seconds_out, u64& tri_tests_out)
	{
		std::vector<ray> probe;
		probe.reserve(rays);

		const u32 res = cam.width();
		for (u32 i = 0; i < rays; ++i)
		{
			// Evenly spaced over the image, deterministically.
			const u64 pixel = (u64(i) * u64(res) * u64(res)) / bvh::max(1u, rays);
			const u32 py = static_cast<u32>(pixel / res) % res;
			const u32 px = static_cast<u32>(pixel % res);
			probe.push_back(cam.generate_ray_through_pixel(px, py));
		}

		std::atomic<u64> found{ 0 };

		timer t;
		parallel_for_range(static_cast<u32>(probe.size()), [&](u32 begin, u32 end) {
			u64 local = 0;
			for (u32 i = begin; i < end; ++i)
			{
				hit h;
				h.t = probe[i].t_max;
				null_stats s;
				if (intersect_brute_force(m, probe[i], h, s)) ++local;
			}
			found.fetch_add(local, std::memory_order_relaxed);
			}, /*chunk*/ 0, threads);
		seconds_out = t.elapsed_s();

		tri_tests_out = u64(probe.size()) * u64(m.triangle_count());
		return seconds_out > 0.0 ? double(tri_tests_out) / seconds_out : 0.0;
	}

} // namespace

bool run_phase0(const phase0_args& args)
{
	LOG_INFO("=== Phase 0 feasibility: %s ===", args.scene_name.c_str());

	std::filesystem::create_directories(args.run_dir);

	const std::string feasibility_csv = args.run_dir + "/phase0_feasibility.csv";
	const std::string collapse_csv = args.run_dir + "/phase0_collapse.csv";
	const std::string cameras_csv = args.run_dir + "/phase0_cameras.csv";

	const u64 rss_start = peak_working_set_bytes();

	// ---------------------------------------------------------------- load
	mesh original;
	timer load_timer;
	if (!original.load_obj(args.scene_path))
	{
		LOG_ERROR("  could not load '%s'", args.scene_path.c_str());
		return false;
	}
	const double load_ms = load_timer.elapsed_ms();
	const u64    rss_after_load = peak_working_set_bytes();

	const f32  diagonal = length(original.bounds().extent());
	const vec3 scene_centre = original.bounds().centroid();

	LOG_INFO("  load: %.1f s, %u triangles, peak RSS %.0f MB, diagonal %g",
		load_ms / 1000.0, original.triangle_count(), mb(rss_after_load), diagonal);

	// ------------------------------------------------------- binary SAH build
	mesh m = original;

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.bins = 32;
	ba.max_leaf_size = 1;
	ba.silent = false;

	bvh2 binary;
	timer build_timer;
	binary.build(m, ba);
	binary.apply_reorder(m);
	binary.refit(m);
	const double build_ms = build_timer.elapsed_ms();
	const u64    rss_after_build = peak_working_set_bytes();

	const u32 binary_nodes = binary.report().node_count;
	const u32 binary_depth = binary.report().max_depth;

	LOG_INFO("  binary SAH: %.1f s, %u nodes, depth %u, peak RSS %.0f MB",
		build_ms / 1000.0, binary_nodes, binary_depth, mb(rss_after_build));

	// The binary tree needs its own stack headroom too.
	const u32 binary_required_stack = required_stack_depth(2u, binary_depth);

	// ------------------------------------------------------- camera screening
	std::vector<candidate_camera> candidates;
	candidates.reserve(args.candidates);

	timer camera_timer;
	for (u32 i = 0; i < args.candidates; ++i)
	{
		candidate_camera c = make_candidate(m, i, diagonal, scene_centre);
		screen_candidate(c, binary, m, args.screen_res, diagonal);
		candidates.push_back(c);
	}
	const double camera_ms = camera_timer.elapsed_ms();

	// Greedily take three accepted views that are pairwise well separated.
	std::vector<u32> chosen;
	for (u32 i = 0; i < candidates.size() && chosen.size() < 3; ++i)
	{
		if (!candidates[i].accepted) continue;

		bool separated = true;
		for (u32 c : chosen)
		{
			if (angle_between_degrees(candidates[i].direction, candidates[c].direction)
				< min_view_separation_degrees)
			{
				separated = false;
				break;
			}
			// Three views from the same spot would not be three views.
			if (length(candidates[i].position - candidates[c].position)
				< f32(min_eye_separation_fraction_of_diagonal) * diagonal)
			{
				separated = false;
				break;
			}
		}
		if (separated) chosen.push_back(i);
	}

	const char* view_names[3] = { "A", "B", "C" };

	u32 accepted_count = 0;
	for (const candidate_camera& c : candidates) if (c.accepted) ++accepted_count;

	LOG_INFO("  cameras: %u candidates screened in %.1f s, %u accepted, %u chosen",
		args.candidates, camera_ms / 1000.0, accepted_count, static_cast<u32>(chosen.size()));

	for (u32 k = 0; k < candidates.size(); ++k)
	{
		const candidate_camera& c = candidates[k];

		i32 view_slot = -1;
		for (u32 v = 0; v < chosen.size(); ++v) if (chosen[v] == k) view_slot = static_cast<i32>(v);

		metrics row;
		row.set("run_id", args.run_id);
		row.set("git_commit", args.git_commit);
		row.set("dirty", args.dirty ? 1u : 0u);
		row.set("scene", args.scene_name);
		row.set("candidate", c.index);
		row.set("view", view_slot >= 0 ? view_names[view_slot] : "");
		row.set("pos_x", double(c.position.x), 6);
		row.set("pos_y", double(c.position.y), 6);
		row.set("pos_z", double(c.position.z), 6);
		row.set("target_x", double(c.target.x), 6);
		row.set("target_y", double(c.target.y), 6);
		row.set("target_z", double(c.target.z), 6);
		row.set("up_x", double(c.up.x), 6);
		row.set("up_y", double(c.up.y), 6);
		row.set("up_z", double(c.up.z), 6);
		row.set("focal", double(c.focal), 3);
		row.set("dir_x", double(c.direction.x), 6);
		row.set("dir_y", double(c.direction.y), 6);
		row.set("dir_z", double(c.direction.z), 6);
		row.set("screen_res", args.screen_res);
		row.set("hit_fraction", c.hit_fraction, 5);
		row.set("median_hit_over_diagonal", c.median_hit_over_diagonal, 5);
		row.set("p10_hit_over_diagonal", c.p10_hit_over_diagonal, 5);
		row.set("accepted", c.accepted ? 1u : 0u);
		row.set("reject_reason", c.reject_reason);
		row.set("scene_diagonal", double(diagonal), 6);
		row.flush(cameras_csv);
	}

	// --------------------------------------------------- oracle throughput
	double oracle_seconds_1 = 0.0, oracle_seconds_n = 0.0;
	double oracle_tps_1 = 0.0, oracle_tps_n = 0.0;
	u64    oracle_tri_tests_1 = 0, oracle_tri_tests_n = 0;
	u32    oracle_rays = args.oracle_probe_rays;

	if (oracle_rays == 0)
	{
		// Keep the probe near a second of single-threaded work regardless of scene
		// size: ~2e8 triangle tests.
		const u64 target_tri_tests = 200000000ull;
		const u64 r = target_tri_tests / bvh::max(1u, original.triangle_count());
		oracle_rays = static_cast<u32>(bvh::max(u32(16), bvh::min(u32(4096), u32(r))));
	}

	{
		const camera probe_cam = chosen.empty()
			? camera::frame_bounds(m.bounds(), 256, 256)
			: camera(256, 256, candidates[chosen[0]].focal,
				candidates[chosen[0]].position, candidates[chosen[0]].target,
				candidates[chosen[0]].up);

		oracle_tps_1 = oracle_throughput(m, probe_cam, oracle_rays, 1u,
			oracle_seconds_1, oracle_tri_tests_1);
		oracle_tps_n = oracle_throughput(m, probe_cam, oracle_rays, args.threads,
			oracle_seconds_n, oracle_tri_tests_n);
	}

	LOG_INFO("  oracle: %u rays, 1 thread %.2f s (%.1f Mtri-tests/s), %u threads %.2f s (%.1f Mtri-tests/s)",
		oracle_rays, oracle_seconds_1, oracle_tps_1 / 1e6,
		args.threads ? args.threads : hardware_threads(), oracle_seconds_n, oracle_tps_n / 1e6);

	// ------------------------------------------------- legacy allocation probe
	// sizeof(u32)*2 + sizeof(decision)*(max_collapse_width+1) per node; decision
	// is {u8,u8,u8,f32}. Computed by the library so it cannot drift from the code.
	u64  legacy_probe_bytes = 0;
	bool legacy_probe_ok = false;
	u64  legacy_probe_peak = 0;

	// --------------------------------------------------------- wide collapses
	bool all_ok = true;

	for (u32 width : widths)
	{
		bvh2 wide = binary;   // fresh copy; collapse mutates in place

		const u64 rss_before = current_working_set_bytes();

		collapse_args ca;
		ca.width = width;
		ca.method = collapse_method::dynamic_programming;
		ca.max_leaf_size = 1;
		ca.silent = true;

		timer collapse_timer;
		const collapse_report cr = collapse(wide, m, ca);
		const double collapse_ms = collapse_timer.elapsed_ms();

		const u64 rss_after = peak_working_set_bytes();

		if (legacy_probe_bytes == 0) legacy_probe_bytes = cr.legacy_scratch_bytes;

		LOG_INFO("  w%-2u: %.1f s, %u nodes, emitted depth %u, fullness %.2f, "
			"scratch %.0f MB (legacy %.0f MB), stack %u/%u %s, peak RSS %.0f MB",
			width, collapse_ms / 1000.0, cr.node_count, cr.max_depth, cr.mean_fullness,
			mb(cr.scratch_bytes), mb(cr.legacy_scratch_bytes),
			cr.required_stack, bvh2_stack_size, cr.stack_bound_ok ? "OK" : "OVERFLOW",
			mb(rss_after));

		if (!cr.stack_bound_ok)
		{
			LOG_ERROR("  w%u requires a stack of %u but bvh2_stack_size is %u",
				width, cr.required_stack, bvh2_stack_size);
			all_ok = false;
		}

		metrics row;
		row.set("run_id", args.run_id);
		row.set("git_commit", args.git_commit);
		row.set("dirty", args.dirty ? 1u : 0u);
		row.set("scene", args.scene_name);
		row.set("triangles", m.triangle_count());
		row.set("width", width);
		row.set("collapse_method", to_string(ca.method));
		row.set("max_leaf_size", ca.max_leaf_size);
		row.set("binary_nodes", binary_nodes);
		row.set("binary_depth", binary_depth);
		row.set("wide_nodes", cr.node_count);
		row.set("wide_interior", cr.interior_count);
		row.set("wide_leaves", cr.leaf_count);
		row.set("emitted_depth", cr.max_depth);
		row.set("mean_fullness", cr.mean_fullness, 4);
		row.set("collapse_s", collapse_ms / 1000.0, 3);
		row.set("scratch_bytes", i64(cr.scratch_bytes));
		row.set("scratch_mb", mb(cr.scratch_bytes), 1);
		row.set("legacy_scratch_bytes", i64(cr.legacy_scratch_bytes));
		row.set("legacy_scratch_mb", mb(cr.legacy_scratch_bytes), 1);
		row.set("scratch_ratio_legacy_over_new", cr.scratch_bytes
			? double(cr.legacy_scratch_bytes) / double(cr.scratch_bytes) : 0.0, 2);
		row.set("required_stack", cr.required_stack);
		row.set("bvh2_stack_size", bvh2_stack_size);
		row.set("stack_bound_ok", cr.stack_bound_ok ? 1u : 0u);
		row.set("wide_bytes", i64(u64(cr.node_count) * sizeof(bvh2_node)));
		row.set("rss_before_mb", mb(rss_before), 1);
		row.set("peak_rss_mb", mb(rss_after), 1);
		row.set("source", "S-measured");
		if (!row.flush(collapse_csv)) all_ok = false;
	}

	// The legacy fixed-table scratch is width independent, so probe it once.
	if (args.probe_legacy_alloc && legacy_probe_bytes)
	{
		LOG_INFO("  probing a %.0f MB contiguous allocation (the legacy DP table)...",
			mb(legacy_probe_bytes));
		legacy_probe_ok = probe_allocation(legacy_probe_bytes, legacy_probe_peak);
		LOG_INFO("  legacy allocation probe: %s (peak RSS %.0f MB)",
			legacy_probe_ok ? "succeeded" : "FAILED", mb(legacy_probe_peak));
	}

	// ------------------------------------------------------------ scene row
	{
		metrics row;
		row.set("run_id", args.run_id);
		row.set("git_commit", args.git_commit);
		row.set("dirty", args.dirty ? 1u : 0u);
		row.set("scene", args.scene_name);
		row.set("scene_path", args.scene_path);
		row.set("triangles", m.triangle_count());
		row.set("scene_diagonal", double(diagonal), 6);
		row.set("load_s", load_ms / 1000.0, 2);
		row.set("build_s", build_ms / 1000.0, 2);
		row.set("binary_nodes", binary_nodes);
		row.set("binary_depth", binary_depth);
		row.set("binary_bytes", i64(u64(binary_nodes) * sizeof(bvh2_node)));
		row.set("binary_required_stack", binary_required_stack);
		row.set("bvh2_stack_size", bvh2_stack_size);
		row.set("rss_start_mb", mb(rss_start), 1);
		row.set("rss_after_load_mb", mb(rss_after_load), 1);
		row.set("rss_after_build_mb", mb(rss_after_build), 1);
		row.set("peak_rss_mb", mb(peak_working_set_bytes()), 1);
		row.set("camera_screen_s", camera_ms / 1000.0, 2);
		row.set("camera_candidates", args.candidates);
		row.set("camera_screen_res", args.screen_res);
		row.set("cameras_accepted", accepted_count);
		row.set("cameras_chosen", static_cast<u32>(chosen.size()));
		row.set("oracle_probe_rays", oracle_rays);
		row.set("oracle_tri_tests", i64(oracle_tri_tests_1));
		row.set("oracle_s_1t", oracle_seconds_1, 3);
		row.set("oracle_mtris_s_1t", oracle_tps_1 / 1e6, 1);
		row.set("oracle_threads", args.threads ? args.threads : hardware_threads());
		row.set("oracle_s_nt", oracle_seconds_n, 3);
		row.set("oracle_mtris_s_nt", oracle_tps_n / 1e6, 1);
		row.set("legacy_probe_bytes", i64(legacy_probe_bytes));
		row.set("legacy_probe_mb", mb(legacy_probe_bytes), 1);
		row.set("legacy_probe_ok", legacy_probe_ok ? 1u : 0u);
		row.set("legacy_probe_peak_rss_mb", mb(legacy_probe_peak), 1);
		row.set("source", "S-measured");
		if (!row.flush(feasibility_csv)) all_ok = false;
	}

	return all_ok;
}
