#pragma once

#include<core/qualifiers.h>

namespace bvh
{
	// per-ray traversal counters

	struct null_stats
	{
		BVH_DEVI void node_step() {}
		BVH_DEVI void prim_step() {}
		BVH_DEVI void box_test() {}
		BVH_DEVI void tri_test() {}
		BVH_DEVI void stack_depth(u32) {}
		BVH_DEVI void touch(const void*, u32) {}
	};

	struct trace_stats
	{
		u32 node_steps{ 0 };
		u32 prim_steps{ 0 };

		u32 box_tests{ 0 };
		u32 tri_tests{ 0 };
		u32 max_stack{ 0 };

		BVH_DEVI void node_step() { ++node_steps; }
		BVH_DEVI void prim_step() { ++prim_steps; }
		BVH_DEVI void box_test() { ++box_tests; }
		BVH_DEVI void tri_test() { ++tri_tests; }
		BVH_DEVI void stack_depth(u32 d) { if (d > max_stack) max_stack = d; }

		BVH_DEVI void touch(const void*, u32) {}

		BVH_DEVI u32 total_steps() const { return node_steps + prim_steps; }
	};

} // namespace bvh

