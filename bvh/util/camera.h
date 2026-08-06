#pragma once

#include <bvh.h>
#include <core/aabb.h>
#include <core/ray.h>
#include <core/rng.h>
#include <core/vec.h>

namespace bvh
{
	// pinhole camera

	class camera
	{

	public:
		camera() = default;

		camera(u32 width, u32 height, f32 focal_length = 50.0f,
			const vec3& position = vec3(0.0f, 0.0f, 1.0f),
			const vec3& target   = vec3(0.0f, 0.0f, 0.0f),
			const vec3& up       = vec3(0.0f, 1.0f, 0.0f));
		
		// auto-frame scene
		static camera frame_bounds(const aabb& bounds, u32 width, u32 height, f32 focal_length = 50.0f, const vec3& dir = vec3(-0.6f, -0.35f, -1.0f), const vec3& up = vec3(0.0f, 1.0f, 0.0f));

		ray generate_ray_through_pixel(u32 i, u32 j) const;

		ray generate_ray_through_pixel(u32 i, u32 j, rng& random) const;

		u32 width() const { return _width; }
		u32 height() const { return _height; }
		vec3 position() const { return _position; }

	private:
		ray make_ray(vec2 uv) const;

		f32 _drdt{ 0.0f };
		vec2 _recip_res{ 0.0f };
		vec3 _position{ 0.0f };
		vec3 _x{ 0.0f };
		vec3 _y{ 0.0f };
		vec3 _z{ 0.0f };
		u32 _width{ 0 };
		u32 _height{ 0 };
	};

}
