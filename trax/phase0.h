#pragma once

#include <bvh.h>

#include <string>

// Phase 0 of PLAN-directional-wide-collapse.md: feasibility measurement,
// instrumentation and camera freeze. It measures; it decides nothing and it
// changes no experiment behaviour.
//
// Per scene it reports:
//   * OBJ load time, triangle count, binary SAH build time, node count, depth;
//   * for each width, the DP collapse time, actual scratch bytes, the legacy
//     fixed-table scratch bytes, emitted node count and depth, mean fullness,
//     required traversal stack depth and whether it fits bvh2_stack_size;
//   * peak working set at each stage, plus an explicit allocation probe of the
//     legacy scratch size so "would it have fit?" is answered empirically;
//   * brute-force oracle throughput, single-threaded and multi-threaded;
//   * screened candidate cameras and the three frozen views.

struct phase0_args
{
	std::string run_id;
	std::string run_dir;      // results/<run_id>
	std::string scene_path;
	std::string scene_name;
	std::string git_commit;
	bool        dirty{ false };

	// Camera screening.
	bvh::u32 screen_res{ 64 };
	bvh::u32 candidates{ 64 };

	// Oracle throughput benchmark.
	bvh::u32 oracle_probe_rays{ 0 };   // 0 selects a per-scene default

	bvh::u32 threads{ 0 };             // 0 = hardware, for the parallel benchmark only
	bool     probe_legacy_alloc{ true };
};

bool run_phase0(const phase0_args& args);
