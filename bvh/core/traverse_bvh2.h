#pragma once

#include <core/bvh2.h>
#include <core/isect.h>
#include <core/mode.h>
#include <core/trace_stats.h>

namespace bvh
{
	// binary bvh traversal

	// Capacity only; this constant never changes a traversal decision.
	// A width-W tree needs 1 + (W-1) * emitted_depth entries in the worst case,
	// because each internal pop replaces one entry with up to W children. San
	// Miguel's max-leaf-size-1 binary SAH tree alone reaches depth 62. The value
	// below is frozen from the Phase-0 measurements in experiments/wide_collapse:
	// the largest requirement measured over the four large scenes at widths 4, 8
	// and 16 is 721 (san-miguel, width 16, emitted depth 48). 1024 leaves headroom.
	// bvh::required_stack_depth() must be checked against it for every tree.
	constexpr u32 bvh2_stack_size = 1024;

	template <typename Mode = default_mode, typename PrimIsect, typename Stats>
	BVH_DEVI bool intersect(const bvh2_view& bvh, const ray& r, hit& h, PrimIsect&& prim, Stats& stats)
	{
		if (bvh.node_count == 0) return false;

		const vec3 inv_d = rcp(r.d);

		struct stack_entry
		{
			f32     t;
			bvh_ptr ptr;
		};

		stack_entry stack[bvh2_stack_size];
		u32         stack_size = 1u;

		stack[0].t = r.t_min;
		stack[0].ptr.is_int = 1;
		stack[0].ptr.child_cnt = 1;
		stack[0].ptr.child_idx = 0;

		bool found_hit = false;
		do
		{
			const stack_entry entry = stack[--stack_size];
			if (entry.t >= h.t) { stats.pruned_pop(); continue; }

			if (entry.ptr.is_int)
			{
				stats.node_step();


				const u32 max_insert_depth = stack_size;

				for (u32 i = 0; i < entry.ptr.child_cnt; ++i)
				{
					const u32 node_id = entry.ptr.child_idx + i;
					stats.box_test();

					const f32 t = intersect<Mode>(bvh.nodes[node_id].bounds, r, inv_d);
					if (t < h.t)
					{
						stats.box_hit();

						u32 j = stack_size++;
						for (; j > max_insert_depth; --j)
						{
							if (stack[j - 1].t > t) break;
							stack[j] = stack[j - 1];
						}

						stack[j].t = t;
						stack[j].ptr = bvh.nodes[node_id].ptr;
					}
				}
				stats.stack_depth(stack_size);
			}
			else
			{
				stats.prim_step();

				for (u32 i = 0; i < entry.ptr.prim_cnt; ++i)
				{
					const u32 prim_id = entry.ptr.prim_idx + i;
					stats.tri_test();
					if (prim(prim_id, r, h))
					{
						h.id = prim_id;
						found_hit = true;
					}
				}
			}
		} while (stack_size);

		return found_hit;
	}

	// Any-hit
	template <typename Mode = default_mode, typename PrimIsect, typename Stats>
	BVH_DEVI bool occluded(const bvh2_view& bvh, const ray& r, PrimIsect&& prim, Stats& stats)
	{
		if (bvh.node_count == 0) return false;

		const vec3 inv_d = rcp(r.d);

		bvh_ptr stack[bvh2_stack_size];
		u32     stack_size = 1u;

		stack[0].is_int = 1;
		stack[0].child_cnt = 1;
		stack[0].child_idx = 0;

		do
		{
			const bvh_ptr ptr = stack[--stack_size];

			if (ptr.is_int)
			{
				stats.node_step();
				for (u32 i = 0; i < ptr.child_cnt; ++i)
				{
					const u32 node_id = ptr.child_idx + i;
					stats.box_test();
					if (intersect<Mode>(bvh.nodes[node_id].bounds, r, inv_d) < r.t_max)
					{
						stats.box_hit();
						stack[stack_size++] = bvh.nodes[node_id].ptr;
					}
				}
				stats.stack_depth(stack_size);
			}
			else
			{
				stats.prim_step();
				for (u32 i = 0; i < ptr.prim_cnt; ++i)
				{
					hit h;
					h.t = r.t_max;
					stats.tri_test();
					if (prim(ptr.prim_idx + i, r, h)) return true;
				}
			}
		} while (stack_size);

		return false;
	}
} // namespace bvh

