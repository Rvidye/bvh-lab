#pragma once

#include<core/aabb.h>
#include<core/vec.h>

namespace bvh 
{
	constexpr f32 t_max_default = 1.0f * (1 << 20);
	constexpr f32 t_min_default = 1.0f / (1 << 5);

	// ray
	struct ray
	{
		vec3 o{ 0.0f };
		f32 t_min{ t_min_default };
		vec3 d{ 0.0f,0.0f,-1.0f };
		f32 t_max{ t_max_default };

		ray() = default;
		BVH_DEVI ray(const vec3& o, const vec3& d, f32 t_min = t_min_default, f32 t_max = t_max_default) : o(o), t_min(t_min), d(d), t_max(t_max) {}
	};

	// hit
	struct hit
	{
		f32 t{ t_max_default };
		vec2 bc{ 0.0f, 0.0f };
		u32 id{ invalid_id };

		hit() = default;
		BVH_DEVI hit(f32 t, const vec2& bc, u32 id) : t(t), bc(bc), id(id) {}

		BVH_DEVI bool valid() const { return id != invalid_id; }
	};

	static_assert(sizeof(ray) == 32);
	static_assert(sizeof(hit) == 16);

	struct alignas(64) triangle
	{
		vec3 vrts[3];

		triangle() = default;
		BVH_DEVI triangle(const vec3& v0, const vec3& v1, const vec3& v2)
		{
			vrts[0] = v0;
			vrts[1] = v1;
			vrts[2] = v2;
		}

		BVH_DEVI aabb bounds() const
		{
			aabb b;
			for (u32 i = 0; i < 3; ++i) b.add(vrts[i]);
			return b;
		}

		BVH_DEVI vec3 normal() const
		{
			return normalize(cross(vrts[0] - vrts[2], vrts[1] - vrts[2]));
		}

		BVH_DEVI vec3 centroid() const { return (vrts[0] + vrts[1] + vrts[2]) * (1.0f / 3.0f); }
	};
} // namespace bvh


