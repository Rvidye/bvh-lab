#include <reference/brute_force.h>

#include <util/parallel.h>
#include <util/stats.h>
#include <util/timer.h>

#include <atomic>

namespace bvh
{
	namespace
	{
		STAT_COUNTER("BruteForce/rays traced", g_rays);
		STAT_COUNTER("BruteForce/triangle tests", g_tri_tests);
		STAT_COUNTER("BruteForce/prim steps", g_prim_steps);
		STAT_PERCENT("BruteForce/rays that hit", g_hit_rays, g_total_rays);

		vec3 background(const vec3& d)
		{
			const f32 t = 0.5f * (d.y + 1.0f);
			return vec3(0.045f, 0.05f, 0.065f) * (1.0f - t) + vec3(0.09f, 0.11f, 0.15f) * t;
		}
	} // namespace

	render_result render_normals(const mesh& m, const camera& cam, image& out, u32 threads)
	{
		const u32 width = cam.width();
		const u32 height = cam.height();

		std::atomic<u64> hit_count{ 0 };

		timer t;
		// one task per scanline
		parallel_for(height, [&](u32 j) 
		{
			u64 row_hits = 0;

			const u32 row = height - 1u - j;

			for (u32 i = 0; i < width; ++i)
			{
				const ray r = cam.generate_ray_through_pixel(i, j);
				hit h;
				trace_stats stats;
				intersect_brute_force(m, r, h, stats);

				++g_rays;
				++g_total_rays;
				g_tri_tests += stats.tri_tests;
				g_prim_steps += stats.prim_steps;

				if (h.valid())
				{
					++row_hits;
					++g_hit_rays;

					vec3 n = m.shading_normal(h.id, h.bc);

					if (dot(n, r.d) > 0.0f) n = -n;
					out.set(i, row, (n + vec3(1.0f)) * 0.5f);
				}
				else
				{
					out.set(i, row, background(normalize(r.d)));
				}
			}
			hit_count.fetch_add(row_hits, std::memory_order_relaxed);
		}, 1, threads);

		render_result result;

		result.seconds = t.elapsed_s();
		result.rays = u64(width) * height;
		result.hits = hit_count.load();
		return result;
	}
} // namespace bvh
