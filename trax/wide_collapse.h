#pragma once

#include <bvh.h>

#include <string>

// Width-8 directional wide-collapse experiment.
//
// Question: does adding workload-directional projected-area information to the
// existing SAH wide-collapse cost produce a width-8 tree that performs less
// traversal work than the ordinary SAH-collapsed width-8 tree?
//
// One binary SAH tree per scene, shared by every variant. Baseline = the
// existing DP collapse. Directional variants = the ratio-of-means projected-area
// weight blended with SAH at lambda 0.25 / 0.5 / 0.75 / 1.0. W is computed from
// view A primary rays; views B and C show whether the effect transfers.

struct wide_collapse_args
{
	std::string run_id;
	std::string run_dir;       // results/<run_id>
	std::string scene_path;
	std::string scene_name;
	std::string cameras_csv;   // experiments/wide_collapse/cameras.csv
	std::string git_commit;
	bool        dirty{ false };

	bvh::u32 width{ 512 };
	bvh::u32 height{ 512 };
	bvh::u32 bins{ 32 };
	bvh::u32 threads{ 0 };     // counters are order-independent sums
};

bool run_wide_collapse(const wide_collapse_args& args);
