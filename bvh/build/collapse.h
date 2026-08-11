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
	//
	// TWO DISTINCT QUANTITIES, previously conflated under the name
	// "mean_overlap":
	//
	// child_area_ratio = sum_i SA(child_i) / SA(parent)
	//     Surface-area EXPANSION, not overlap. Perfectly disjoint children still
	//     produce a large value -- splitting a cube in half gives ~1.33 with
	//     zero overlap -- and the value grows with width simply because there
	//     are more boxes. Useful as a proxy for how much box surface a ray must
	//     test, but it is NOT geometric overlap and must not be read as such.
	//
	// pair_overlap = SA(intersection(child_i, child_j)) / SA(parent), over all
	//     i < j. This IS geometric overlap: it is zero for disjoint children and
	//     rises only when sibling volumes genuinely intersect. Overlap is what
	//     forces a ray into more than one child, so this is the quantity that
	//     explains extra traversal.
	//
	// NORMALIZATION. The number of pairs is C(n,2), which grows quadratically
	// with width: 1 pair at width 2, 6 at width 4, 28 at width 8. Therefore:
	//   - `mean_pair_overlap` divides by the pair count and IS comparable
	//     across widths;
	//   - `sum_pair_overlap` is NOT comparable across widths and is retained
	//     only so the mean can be recomputed;
	//   - `pair_count` is recorded so a reader can tell which is which.
	struct depth_overlap_stats
	{
		u32    internal_nodes{0};
		u64    pair_count{0};

		double mean_child_area_ratio{0.0};

		double sum_pair_overlap{0.0};  // width-DEPENDENT; see note above
		double mean_pair_overlap{0.0}; // width-normalized
		double p95_pair_overlap{0.0};
		double max_pair_overlap{0.0};
	};

	struct overlap_profile
	{
		// Deeper than the 32 used before: a binary build on a large scene can
		// exceed it, and silently dropping those depths biased the profile
		// toward the shallow levels.
		static constexpr u32 max_depth_buckets = 64;

		// Every member zero-initialized. The previous version left `nodes`
		// uninitialized, so every count -- and therefore every derived mean --
		// started from garbage.
		depth_overlap_stats depth[max_depth_buckets]{};
		u32                 depth_count{0};

		// Nodes deeper than max_depth_buckets, counted rather than ignored so a
		// truncated profile is visible instead of silent.
		u32 nodes_beyond_buckets{0};
	};

	overlap_profile compute_overlap_profile(const bvh2& tree);

	// Surface area of the intersection of two boxes; 0 when they are disjoint on
	// any axis. Exposed for testing.
	f32 box_intersection_area(const aabb& a, const aabb& b);
} // namespace bvh
