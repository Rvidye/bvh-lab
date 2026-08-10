#pragma once

#include <core/rng.h>
#include <core/vec.h>

namespace bvh {

	// Sampling helpers for ray-set generation.

	constexpr f32 pi_v = 3.14159265358979323846f;

	BVH_DEVI void orthonormal_basis(const vec3& n, vec3& t, vec3& b)
	{
		const f32 sign = n.z >= 0.0f ? 1.0f : -1.0f;
		const f32 a = -1.0f / (sign + n.z);
		const f32 c = n.x * n.y * a;

		t = vec3(1.0f + sign * n.x * n.x * a, sign * c, -sign * n.x);
		b = vec3(c, sign + n.y * n.y * a, -n.y);
	}

	// Cosine-weighted hemisphere around 'n', via concentric disk mapping.
	BVH_DEVI vec3 cosine_hemisphere(const vec3& n, f32 u1, f32 u2)
	{
		const f32 a = 2.0f * u1 - 1.0f;
		const f32 b = 2.0f * u2 - 1.0f;

		f32 r, phi;
		if (a == 0.0f && b == 0.0f)
		{
			r = 0.0f;
			phi = 0.0f;
		}
		else if (a * a > b * b)
		{
			r = a;
			phi = (pi_v / 4.0f) * (b / a);
		}
		else
		{
			r = b;
			phi = (pi_v / 2.0f) - (pi_v / 4.0f) * (a / b);
		}

		const f32 x = r * cosf(phi);
		const f32 y = r * sinf(phi);
		const f32 z = sqrtf(bvh::max(0.0f, 1.0f - x * x - y * y));

		vec3 tangent, bitangent;
		orthonormal_basis(n, tangent, bitangent);
		return tangent * x + bitangent * y + n * z;
	}

	// Uniform direction on the sphere.
	BVH_DEVI vec3 uniform_sphere(f32 u1, f32 u2)
	{
		const f32 z = 1.0f - 2.0f * u1;
		const f32 r = sqrtf(bvh::max(0.0f, 1.0f - z * z));
		const f32 phi = 2.0f * pi_v * u2;
		return vec3(r * cosf(phi), r * sinf(phi), z);
	}

} // namespace bvh
