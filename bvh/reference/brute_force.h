#pragma once

#include <bvh.h>
#include <core/isect.h>
#include <core/mode.h>
#include <core/trace_stats.h>
#include <util/camera.h>
#include <util/image.h>
#include <util/mesh.h>

#include <vector>

namespace bvh
{
	// linear scan over all triangles, slow but will confirm correctness of all the other bvh's

	template <typename Mode = default_mode,typename Stats>
	bool intersect_brute_force(const mesh& m, const ray& r, hit& h, Stats& stats)
	{
		bool found = false;
		const u32 count = m.triangle_count();

		stats.prim_step(); // technically we are traversing a tree with 1 node :-P
		for (u32 i = 0; i < count; ++i)
		{
			stats.tri_test();
			if (intersect<Mode>(m.get_triangle(i), r, h))
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

	template <typename Mode = default_mode,typename Stats>
	bool occluded_brute_force(const mesh& m, const ray& r, Stats& stats)
	{
		const u32 count = m.triangle_count();
		stats.prim_step();
		for (u32 i = 0; i < count; ++i)
		{
			stats.tri_test();
			hit h;
			h.t = r.t_max;
			if(intersect<Mode>(m.get_triangle(i), r, h)) return true;
		}
		return false;
	}

	// primitive-id equality is not a valid correctness criterion in general

	constexpr f32 oracle_t_rel_tolerance = 1e-5f;

	inline bool t_agrees(f32 a, f32 b, f32 rel = oracle_t_rel_tolerance)
	{
		if (a == b) return true;
		const f32 d = a > b ? a - b : b - a;
		const f32 aa = a > 0.0f ? a : -a;
		const f32 ab = b > 0.0f ? b : -b;
		const f32 m = max(max(aa, ab), 1.0f);
		return d <= rel * m;
	}

	template <typename Mode = default_mode>
	inline void nearest_tie_set(const mesh& m, const ray& r, f32 t, std::vector<u32>& out, f32 rel = oracle_t_rel_tolerance)
	{
		out.clear();
		const u32 count = m.triangle_count();
		for(u32 i = 0; i < count; ++i)
		{
			hit probe;
			probe.t = r.t_max;
			if (intersect<Mode>(m.get_triangle(i), r, probe) && t_agrees(probe.t, t, rel))
				out.push_back(i);
		}
	}

	struct oracle_result
	{
		bool agree{ true };
		bool miss_mismatch{ false }; // one found a hit and the other did not
		bool t_mismatch{ false };
		bool id_mismatch{ false };   // ids differed AND the nearest hit was unique
		bool tie_resolved{ false };  // ids differed but both are in the tie set
		f32  t_delta{ 0.0f };
	};

	template<typename Mode = default_mode>
	inline oracle_result compare_against_oracle(const mesh& m, const ray& r, bool candidate_hit, f32 candidate_t, u32 candidate_id, std::vector<u32>& scratch)
	{
		oracle_result res;

		hit ref;
		ref.t = r.t_max;
		null_stats s;
		const bool ref_hit = intersect_brute_force<Mode>(m, r, ref, s);

		if (candidate_hit != ref_hit)
		{
			res.agree = false;
			res.miss_mismatch = true;
			return res;
		}
		if (!ref_hit) return res;

		res.t_delta = candidate_t > ref.t ? candidate_t - ref.t : ref.t - candidate_t;
		if (!t_agrees(candidate_t, ref.t))
		{
			res.agree = false;
			res.t_mismatch = true;
			return res;
		}

		if (candidate_id == ref.id) return res;

		// Ids differ. Legitimate only if several primitives tie at this distance.
		nearest_tie_set<Mode>(m, r, ref.t, scratch);

		bool in_set = false;
		for (u32 id : scratch)
			if (id == candidate_id) { in_set = true; break; }

		if (in_set && scratch.size() > 1) res.tie_resolved = true;
		else { res.agree = false; res.id_mismatch = true; }

		return res;
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
