#pragma once

#include <build/bvh2_builder.h>
#include <core/mode.h>
#include <eval/rayset.h>
#include <core/traverse_bvh2.h>
#include <util/camera.h>
#include <util/image.h>
#include <util/mesh.h>

#include <vector>

namespace bvh {

	// Mesh-backed primitive intersection for the generic traversal.
	template <typename Mode = default_mode>
	struct mesh_prims
	{
		const mesh* m{ nullptr };
		const u32* slots{ nullptr };

		BVH_DEVI bool operator()(u32 slot, const ray& r, hit& h) const
		{
			return intersect<Mode>(m->get_triangle(slots[slot]), r, h);
		}
	};

	template <typename Mode = default_mode>
	inline mesh_prims<Mode> make_prims(const mesh& m, const bvh2& tree)
	{
		return mesh_prims<Mode>{&m, tree.prim_indices().data()};
	}

	struct trace_result
	{
		double seconds{ 0.0 };
		u64    rays{ 0 };
		u64    hits{ 0 };

		u64 node_steps{ 0 };
		u64 prim_steps{ 0 };

		u64 box_tests{ 0 };
		u64 tri_tests{ 0 };
		u32 max_stack{ 0 };

		u32    reps{ 1 };
		double seconds_min{ 0.0 };
		double seconds_stddev{ 0.0 };

		double mrays_per_second() const { return seconds > 0.0 ? double(rays) / seconds / 1e6 : 0.0; }
		double cv() const { return seconds > 0.0 ? seconds_stddev / seconds : 0.0; }
		double node_steps_per_ray() const { return rays ? double(node_steps) / double(rays) : 0.0; }
		double prim_steps_per_ray() const { return rays ? double(prim_steps) / double(rays) : 0.0; }
		double tri_tests_per_ray() const { return rays ? double(tri_tests) / double(rays) : 0.0; }
	};
	trace_result render_bvh2(const bvh2& tree, const mesh& m, const camera& cam, image& out, std::vector<u32>* counts = nullptr, u32 threads = 0);
	trace_result trace_only(const bvh2& tree, const mesh& m, const camera& cam, u32 threads = 0);
	trace_result trace_rayset(const bvh2& tree, const mesh& m, const rayset& rays, u32 threads = 0, u32 reps = 1);
	trace_result occlude_rayset(const bvh2& tree, const mesh& m, const rayset& rays, u32 threads = 0, u32 reps = 1);
} // namespace bvh
