#include <eval/trace.h>

#include <util/parallel.h>
#include <util/stats.h>
#include <util/timer.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <cmath>
#include <mutex>
#include <vector>

namespace bvh {

	namespace {

		STAT_COUNTER("BVH2/node steps", g_node_steps);
		STAT_COUNTER("BVH2/prim steps", g_prim_steps);
		STAT_COUNTER("BVH2/box tests", g_box_tests);
		STAT_COUNTER("BVH2/triangle tests", g_tri_tests);
		STAT_DISTRIBUTION("BVH2/steps per ray", g_steps_per_ray);
		STAT_PERCENT("BVH2/rays that hit", g_hit_rays, g_total_rays);

		vec3 background(const vec3& d)
		{
			const f32 t = 0.5f * (d.y + 1.0f);
			return vec3(0.045f, 0.05f, 0.065f) * (1.0f - t) + vec3(0.09f, 0.11f, 0.15f) * t;
		}

		struct thread_totals
		{
			u64 node_steps{ 0 }, prim_steps{ 0 }, box_tests{ 0 }, tri_tests{ 0 }, hits{ 0 };
			u32 max_stack{ 0 };

			void merge(const trace_stats& s)
			{
				node_steps += s.node_steps;
				prim_steps += s.prim_steps;
				box_tests += s.box_tests;
				tri_tests += s.tri_tests;
				if (s.max_stack > max_stack) max_stack = s.max_stack;
			}
		};

		struct shared_totals
		{
			std::mutex    mutex;
			thread_totals total;

			void merge(const thread_totals& t)
			{
				std::lock_guard<std::mutex> lock(mutex);
				total.node_steps += t.node_steps;
				total.prim_steps += t.prim_steps;
				total.box_tests += t.box_tests;
				total.tri_tests += t.tri_tests;
				total.hits += t.hits;
				if (t.max_stack > total.max_stack) total.max_stack = t.max_stack;
			}
		};

		trace_result finish(const shared_totals& shared, u64 rays, double seconds)
		{
			trace_result r;
			r.seconds = seconds;
			r.rays = rays;
			r.hits = shared.total.hits;
			r.node_steps = shared.total.node_steps;
			r.prim_steps = shared.total.prim_steps;
			r.box_tests = shared.total.box_tests;
			r.tri_tests = shared.total.tri_tests;
			r.max_stack = shared.total.max_stack;
			return r;
		}

	} // namespace

	trace_result render_bvh2(const bvh2& tree, const mesh& m, const camera& cam, image& out,
		std::vector<u32>* counts, u32 threads)
	{
		const u32 width = cam.width();
		const u32 height = cam.height();

		const bvh2_view  view = tree.view();
		const auto prims = make_prims(m, tree);

		if (counts) counts->assign(size_t(width) * height, 0u);

		shared_totals shared;
		timer         t;

		parallel_for(height, [&](u32 j) {
			thread_totals local;

			const u32 row = height - 1u - j;

			for (u32 i = 0; i < width; ++i)
			{
				const ray r = cam.generate_ray_through_pixel(i, j);

				hit         h;
				trace_stats stats;
				intersect(view, r, h, prims, stats);

				local.merge(stats);
				++g_total_rays;
				g_node_steps += stats.node_steps;
				g_prim_steps += stats.prim_steps;
				g_box_tests += stats.box_tests;
				g_tri_tests += stats.tri_tests;
				g_steps_per_ray << i64(stats.total_steps());

				if (counts) (*counts)[size_t(row) * width + i] = stats.total_steps();

				if (h.valid())
				{
					++local.hits;
					++g_hit_rays;

					vec3 n = m.shading_normal(tree.prim_index(h.id), h.bc);
					if (dot(n, r.d) > 0.0f) n = -n;
					out.set(i, row, (n + vec3(1.0f)) * 0.5f);
				}
				else
				{
					out.set(i, row, background(normalize(r.d)));
				}
			}

			shared.merge(local);
			}, /*chunk*/ 1, threads);

		return finish(shared, u64(width) * height, t.elapsed_s());
	}

	static trace_result trace_rayset_once(const bvh2&, const mesh&, const rayset&, u32);
	static trace_result occlude_rayset_once(const bvh2&, const mesh&, const rayset&, u32);

	namespace {

		trace_result repeat(u32 reps, const std::function<trace_result()>& run)
		{
			if (reps <= 1)
			{
				trace_result r = run();
				r.reps = 1;
				r.seconds_min = r.seconds;
				r.seconds_stddev = 0.0;
				return r;
			}

			std::vector<double> times;
			times.reserve(reps);
			trace_result first = run();   // also serves as warm-up for the rest
			times.push_back(first.seconds);

			for (u32 i = 1; i < reps; ++i) times.push_back(run().seconds);

			std::sort(times.begin(), times.end());

			double sum = 0.0;
			for (double t : times) sum += t;
			const double mean = sum / double(times.size());

			double var = 0.0;
			for (double t : times) var += (t - mean) * (t - mean);
			var /= double(times.size());

			trace_result r = first;      // step counts are deterministic; keep them
			r.reps = reps;
			r.seconds = times[times.size() / 2];
			r.seconds_min = times.front();
			r.seconds_stddev = std::sqrt(var);
			return r;
		}

	} // namespace

	trace_result trace_rayset(const bvh2& tree, const mesh& m, const rayset& rays, u32 threads, u32 reps)
	{
		return repeat(reps, [&] { return trace_rayset_once(tree, m, rays, threads); });
	}

	trace_result occlude_rayset(const bvh2& tree, const mesh& m, const rayset& rays, u32 threads, u32 reps)
	{
		return repeat(reps, [&] { return occlude_rayset_once(tree, m, rays, threads); });
	}

	static trace_result trace_rayset_once(const bvh2& tree, const mesh& m, const rayset& rays, u32 threads)
	{
		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		shared_totals shared;
		timer         t;
		const u32 n = rays.size();
		parallel_for_range(n, [&](u32 begin, u32 end) {
			thread_totals local;

			for (u32 i = begin; i < end; ++i)
			{
				const ray r = rays.get(i);

				hit h;
				h.t = r.t_max;
				trace_stats stats;
				intersect(view, r, h, prims, stats);

				local.merge(stats);
				if (h.valid()) ++local.hits;
			}

			shared.merge(local);
			}, /*chunk*/ 0, threads);

		return finish(shared, n, t.elapsed_s());
	}

	static trace_result occlude_rayset_once(const bvh2& tree, const mesh& m, const rayset& rays, u32 threads)
	{
		const bvh2_view view = tree.view();
		const auto      prims = make_prims(m, tree);

		shared_totals shared;
		timer         t;

		const u32 n = rays.size();
		parallel_for_range(n, [&](u32 begin, u32 end) {
			thread_totals local;

			for (u32 i = begin; i < end; ++i)
			{
				trace_stats stats;
				if (occluded(view, rays.get(i), prims, stats)) ++local.hits;
				local.merge(stats);
			}

			shared.merge(local);
			}, /*chunk*/ 0, threads);

		return finish(shared, n, t.elapsed_s());
	}

	trace_result trace_only(const bvh2& tree, const mesh& m, const camera& cam, u32 threads)
	{
		const u32 width = cam.width();
		const u32 height = cam.height();

		const bvh2_view  view = tree.view();
		const auto prims = make_prims(m, tree);

		shared_totals shared;
		timer         t;

		parallel_for(height, [&](u32 j) {
			thread_totals local;

			for (u32 i = 0; i < width; ++i)
			{
				const ray r = cam.generate_ray_through_pixel(i, j);

				hit         h;
				trace_stats stats;
				intersect(view, r, h, prims, stats);

				local.merge(stats);
				if (h.valid()) ++local.hits;
			}

			shared.merge(local);
			}, /*chunk*/ 1, threads);

		return finish(shared, u64(width) * height, t.elapsed_s());
	}

} // namespace bvh
