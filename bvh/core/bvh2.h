#pragma once

#include <core/aabb.h>
#include <core/qualifiers.h>

namespace bvh
{
	union bvh_ptr
	{
		struct
		{
			u32 is_int : 1;
			u32 prim_cnt : 5;
			u32 prim_idx : 26;
		};
		struct
		{
			u32 : 1;
			u32 child_cnt : 5;
			u32 child_idx : 26;
		};
		u32 raw;

		bvh_ptr() : raw(0) {}
	};

	struct alignas(32) bvh2_node
	{
		aabb bounds;
		bvh_ptr ptr;
	};

	static_assert(sizeof(bvh_ptr) == 4);
	static_assert(sizeof(bvh2_node) == 32);

	// flat POD view of binary BVH
	struct bvh2_view
	{
		const bvh2_node* nodes{ nullptr };
		u32 node_count{ 0 };
	};

} // namespace bvh
