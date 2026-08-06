#include <util/camera.h>
#include <util/check.h>

namespace bvh 
{
	camera::camera(u32 width, u32 height, f32 focal_length, const vec3& position, const vec3& target, const vec3& up) : _width(width), _height(height)
	{
		CHECK_GT(width, 0u);
		CHECK_GT(height, 0u);
		CHECK_GT(focal_length, 0.0f);

		const f32 aspect = static_cast<f32>(width) / static_cast<f32>(height);

		_recip_res.x = 1.0f / static_cast<f32>(width);
		_recip_res.y = 1.0f / static_cast<f32>(height);

		_drdt = 24.0f / focal_length / static_cast<f32>(height);
		_drdt = sqrtf(_drdt * _drdt / 3.1415926f);

		_position = position;

		const vec3 direction = target - position;
		_z = -normalize(direction);
		_x = normalize(cross(up, _z));
		_y = cross(_z, _x);

		_x *= 24.0f / focal_length * aspect;
		_y *= 24.0f / focal_length;
	}

	camera camera::frame_bounds(const aabb& bounds, u32 width, u32 height, f32 focal_length, const vec3& dir, const vec3& up)
	{
		const vec3 centre = bounds.centroid();
		const f32 radius = bvh::max(0.5f * length(bounds.extent()), 1e-6f);

		const f32 tan_half_fov = 12.0f / focal_length;
		const f32 distance = 1.25f * radius / tan_half_fov;

		const vec3 position = centre - normalize(dir) * distance;
		return camera(width, height, focal_length, position, centre, up);
	}

	ray camera::make_ray(vec2 uv) const
	{
		uv *= _recip_res;
		uv -= vec2(0.5f);

		ray r;
		r.o = _position;
		r.d = _x * uv.x + _y * uv.y - _z;
		r.t_min = t_min_default;
		r.t_max = t_max_default;
		return r;
	}

	ray camera::generate_ray_through_pixel(u32 i, u32 j) const
	{
		DCHECK_LT(i, _width);
		DCHECK_LT(j, _height);
		return make_ray(vec2(static_cast<f32>(i),static_cast<f32>(j)) + vec2(0.5f));
	}

	ray camera::generate_ray_through_pixel(u32 i, u32 j, rng& random) const
	{
		DCHECK_LT(i, _width);
		DCHECK_LT(j, _height);
		const vec2 jitter(random.next_f32(), random.next_f32());
		return make_ray(vec2(static_cast<f32>(i), static_cast<f32>(j)) + jitter);
	}
} // namespace bvh
