#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <util/mesh.h>

namespace bvh
{
	constexpr u32 max_collapse_width = 31;

	enum class collapse_method
	{
		greedy, // expand child in the current set which has largest surface area.
		dynamic_programming, // Ylitie et al. 2017: minimising SAH over the whole tree, optimal for fixed leaf policy.
	};

	const char* to_string(collapse_method m);

	struct collapse_args
	{
		u32 width{ 8 };
		collapse_method method{ collapse_method::dynamic_programming };
		u32 max_leaf_size{ 1 };
		//SAH constant for the DP cost model
		f32 c_traversal{ 1.0f };
		f32 c_intersect{ 1.0f };
		bool silent{ false };
	};

	struct collapse_report
	{
		double collapse_ms{ 0.0 };
		u32 width{ 2 };
		u32 node_count{ 0 };
		u32 interior_count{ 0 };
		u32 leaf_count{ 0 };
		u32 max_depth{ 0 };
		// mean children per interior mode
		double mean_fullness{ 0.0 };
		// histogram[c] = interior nodes with exactly c children
		u32 fullness_histogram[max_collapse_width + 1]{};
	};

	collapse_report collapse(bvh2& tree, const mesh& m, const collapse_args& args = {});

	// Per-depth structural statistics.
	struct depth_overlap_stats
	{
		u32    internal_nodes{0};
		u64    pair_count{0};

		double mean_child_area_ratio{0.0};

		double sum_pair_overlap{0.0};
		double mean_pair_overlap{0.0};
		double p95_pair_overlap{0.0};
		double max_pair_overlap{0.0};
	};

	struct overlap_profile
	{
		static constexpr u32 max_depth_buckets = 64;
		depth_overlap_stats depth[max_depth_buckets]{};
		u32                 depth_count{0};
		u32 nodes_beyond_buckets{0};
	};

	overlap_profile compute_overlap_profile(const bvh2& tree);
	f32 box_intersection_area(const aabb& a, const aabb& b);

} // namespace bvh
