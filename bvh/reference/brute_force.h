#pragma once

#include <bvh.h>
#include <core/isect.h>
#include <core/trace_stats.h>
#include <util/camera.h>
#include <util/image.h>
#include <util/mesh.h>

#include <vector>

namespace bvh
{
	// linear scan over all triangles, slow but will confirm correctness of all the other bvh's

	template <typename Stats>
	bool intersect_brute_force(const mesh& m, const ray& r, hit& h, Stats& stats)
	{
		bool found = false;
		const u32 count = m.triangle_count();

		stats.prim_step(); // technically we are traversing a tree with 1 node :-P
		for (u32 i = 0; i < count; ++i)
		{
			stats.tri_test();
			if (intersect(m.get_triangle(i), r, h))
			{
				h.id = i;
				found = true;
			}
		}
		return found;
	}

	inline bool intersect_brute_force(const mesh& m, const ray& r, hit& h)
	{
		null_stats stats;
		return intersect_brute_force(m, r, h, stats);
	}

	template <typename Stats>
	bool occluded_brute_force(const mesh& m, const ray& r, Stats& stats)
	{
		const u32 count = m.triangle_count();
		stats.prim_step();
		for (u32 i = 0; i < count; ++i)
		{
			stats.tri_test();
			hit h;
			h.t = r.t_max;
			if(intersect(m.get_triangle(i), r, h)) return true;
		}
		return false;
	}

	struct render_result
	{
		double seconds{ 0.0f };
		u64 rays{0};
		u64 hits{0};

		double mrays_per_second() const
		{
			return seconds > 0.0 ? double(rays) / seconds / 1e6 : 0.0;
		}
	};

	render_result render_normals(const mesh& m, const camera& cam, image& out, u32 threads = 0);
} // namespace bvh
