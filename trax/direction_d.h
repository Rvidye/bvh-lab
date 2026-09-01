#pragma once

#include <bvh.h>
#include <util/mesh.h>

#include <string>

// Direction D runner. Deliberately separate from the ordinary M1/M3 flow in
// main.cpp: --direction_d adds this pass, --direction_d_only runs nothing else,
// and neither is reachable from a plain trax invocation.
//
// The frozen experiment contract is experiments/direction_d/README.md.

struct direction_d_args
{
	std::string run_id;
	std::string run_dir;      // results/<run_id>
	std::string scene_path;
	std::string scene_name;
	std::string git_commit;   // recorded verbatim into every row
	bool        dirty{ false };

	bvh::u32 width{ 512 };
	bvh::u32 height{ 512 };
	bvh::u32 bins{ 32 };
	bvh::u32 incoherent_count{ 262144 };

	// The first experiment is single threaded so the CSVs are deterministic.
	bvh::u32 threads{ 1 };

	bool validate{ true };
};

// Returns false if any coordinate failed to build, validate or write. A failed
// coordinate never leaves a row behind.
bool run_direction_d(const bvh::mesh& original, const direction_d_args& args);
