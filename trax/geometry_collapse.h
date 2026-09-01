#pragma once

#include <bvh.h>

#include <string>

// Geometry-derived directional wide-collapse experiment.
//
// Hypothesis: starting from one ordinary binary SAH tree, does adding a
// geometry-derived axis-projected emptiness loss to the ordinary SAH collapse
// cost select a better set and grouping of BVH8 children?
//
// NO RAYS ARE USED TO CONSTRUCT ANY TREE. Every tree is built before a single
// evaluation ray exists. The directional information comes exclusively from the
// triangles inside each binary subtree. Cameras and rays are evaluation inputs
// only.

struct geometry_collapse_args
{
	std::string run_id;
	std::string run_dir;       // results/<run_id>
	std::string scene_path;
	std::string scene_name;
	std::string cameras_csv;   // experiments/wide_collapse/cameras.csv
	std::string image_dir;     // where the native heatmaps are written
	std::string git_commit;
	bool        dirty{ false };

	bvh::u32 width{ 512 };
	bvh::u32 height{ 512 };
	bvh::u32 bins{ 32 };
	bvh::u32 threads{ 0 };     // counters are order-independent sums
};

bool run_geometry_collapse(const geometry_collapse_args& args);
