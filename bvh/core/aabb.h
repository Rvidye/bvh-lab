#pragma once

#include<core/vec.h>

namespace bvh 
{
	// axis-aligned bounding box
	struct aabb
	{
		vec3 min{  FLT_MAX };
		vec3 max{ -FLT_MAX };

		aabb() = default;
		BVH_DEVI aabb(const vec3& lo, const vec3& hi) : min(lo), max(hi) {}

		BVH_DEVI void add(const aabb& other)
		{
			min = bvh::min(min, other.min);
			max = bvh::max(max, other.max);
		}

		BVH_DEVI void add(const vec3& point)
		{
			min = bvh::min(min, point);
			max = bvh::max(max, point);
		}

		BVH_DEVI f32 surface_area() const
		{
			if (min.x > max.x) return 0.0f;

			const f32 x = max.x - min.x;
			const f32 y = max.y - min.y;
			const f32 z = max.z - min.z;

			return (x * y + y * z + z * x) * 2.0f;
		}

		BVH_DEVI vec3 centroid() const { return (min + max) * 0.5f; }

		BVH_DEVI vec3 extent() const { return empty() ? vec3(0.0f) : max - min; }

		BVH_DEVI bool empty() const { return min.x > max.x; }

		BVH_DEVI u32 longest_axis() const
		{
			u32 axis = 0;
			f32 max_length = 0.0f;
			for (u32 i = 0; i < 3; i++)
			{
				const f32 length = max[i] - min[i];
				if (length > max_length)
				{
					axis = i;
					max_length = length;
				}
			}
			return axis;
		}
	};

	static_assert(sizeof(aabb) == 24);

	BVH_DEVI aabb merge(const aabb& a, const aabb& b)
	{
		aabb r = a;
		r.add(b);
		return r;
	}
} // namespace bvh
