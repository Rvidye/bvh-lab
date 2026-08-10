#pragma once

#include<core/aabb.h>
#include<core/mode.h>
#include<core/ray.h>

namespace bvh
{

#ifndef BVH_ROBUST_SLAB
#define BVH_ROBUST_SLAB 0
#endif // !BVH_ROBUST_SLAB

constexpr f32 robust_tmax_scale = 1.00000024f;

template <typename Mode = default_mode>
BVH_DEVI f32 intersect(const aabb& box, const ray& r, const vec3& inv_d)
{
	if (box.max.x < box.min.x) return r.t_max; // degenerate box

	vec3 t0 = (box.min - r.o) * inv_d;
	vec3 t1 = (box.max - r.o) * inv_d;

	if constexpr (Mode::axis_parallel_guard)
	{
		for (u32 a = 0; a < 3; ++a) 
		{
			if (r.d[a] != 0.0f) continue;

			const bool inside = r.o[a] >= box.min[a] && r.o[a] <= box.max[a];
			t0[a] = inside ? -INFINITY : INFINITY;
			t1[a] = inside ? INFINITY : INFINITY;
		}
	}
	const vec3 tminv = bvh::min(t0, t1);
	const vec3 tmaxv = bvh::max(t0, t1);

	const f32 tmin = bvh::max(bvh::max(tminv.x, tminv.y), bvh::max(tminv.z, r.t_min));
	f32 tmax = bvh::min(bvh::min(tmaxv.x, tmaxv.y), bvh::min(tmaxv.z, r.t_max));

	if constexpr (Mode::slab_padding) tmax *= robust_tmax_scale;
	if (tmin > tmax || tmax <= r.t_min) return r.t_max; // no hit, or behind
	return tmin;
}

template <typename Mode = default_mode>
BVH_DEVI bool intersect(const triangle& tri, const ray& r, hit& h)
{
	const vec3 e0 = tri.vrts[1] - tri.vrts[2];
	const vec3 e1 = tri.vrts[0] - tri.vrts[2];

	const vec3 r1 = cross(r.d, e0);
	const f32 denom = dot(e1, r1);

	if constexpr (Mode::reject_degenerate)
	{
		if (!(denom > degenerate_det_epsilon || denom < -degenerate_det_epsilon))
			return false;
	}

	const f32 rcp_denom = 1.0f / denom;
	const vec3 s = r.o - tri.vrts[2];

	const f32 b1 = dot(s, r1) * rcp_denom;
	if (b1 < 0.0f || b1 > 1.0f) return false;

	const vec3 r2 = cross(s, e1);
	const f32 b2 = dot(r.d, r2) * rcp_denom;
	if (b2 < 0.0f || (b2 + b1) > 1.0f) return false;

	const f32 t = dot(e0, r2) * rcp_denom;
	if (t < r.t_min || t > h.t) return false;

	if constexpr (Mode::reject_degenerate)
	{
		if (!(t == t)) return false;
	}

	h.bc = vec2(b1, b2);
	h.t = t;
	return true;
}

} // namespace bvh
