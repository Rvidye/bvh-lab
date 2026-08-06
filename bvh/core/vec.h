#pragma once

#include<core/qualifiers.h>

#include<cfloat>
#include<cmath>

namespace bvh 
{

	// common maths functions

	BVH_DEVI f32 min(f32 a, f32 b) { return (b < a) ? b : a; }
	BVH_DEVI f32 max(f32 a, f32 b) { return (a < b) ? b : a; }

	BVH_DEVI u32 min(u32 a, u32 b) { return (b < a) ? b : a; }
	BVH_DEVI u32 max(u32 a, u32 b) { return (a < b) ? b : a; }

	BVH_DEVI f32 abs(f32 a) { return a > 0.0f ? a : -a; }
	BVH_DEVI f32 clamp(f32 a, f32 lo, f32 hi) { return min(max(a, lo), hi); }

	// vec2

	struct vec2
	{
		union
		{
			f32 e[2];
			struct { f32 x, y; };
		};

		vec2() : e{ 0.0f, 0.0f } {}
		BVH_DEVI explicit vec2(f32 s) : e{ s,s } {}
		BVH_DEVI vec2(f32 x, f32 y) : e{ x,y } {}
		
		BVH_DEVI f32& operator[](u32 i) { return e[i]; }
		BVH_DEVI const f32& operator[](u32 i) const { return e[i]; }
	};

	BVH_DEVI vec2 operator+(const vec2& a, const vec2& b) { return vec2(a.x + b.x, a.y + b.y); }
	BVH_DEVI vec2 operator-(const vec2& a, const vec2& b) { return vec2(a.x - b.x, a.y - b.y); }
	BVH_DEVI vec2 operator*(const vec2& a, const vec2& b) { return vec2(a.x * b.x, a.y * b.y); }
	BVH_DEVI vec2 operator*(const vec2& a, f32 s) { return vec2(a.x * s, a.y * s); }
	BVH_DEVI vec2& operator+=(vec2& a, const vec2& b) { a = a + b; return a; }
	BVH_DEVI vec2& operator-=(vec2& a, const vec2& b) { a = a - b; return a; }
	BVH_DEVI vec2& operator*=(vec2& a, const vec2& b) { a = a * b; return a; }

	// vec3
	
	struct vec3
	{
		union
		{
			f32 e[3];
			struct { f32 x, y, z; };
			struct { f32 r, g, b; };
		};

		vec3() : e{ 0.0f, 0.0f, 0.0f } {}
		BVH_DEVI explicit vec3(f32 s) : e{ s, s, s } {}
		BVH_DEVI vec3(f32 x, f32 y, f32 z) : e{ x, y, z } {}

		BVH_DEVI f32& operator[](u32 i) { return e[i]; }
		BVH_DEVI const f32& operator[](u32 i) const { return e[i]; }

		BVH_DEVI vec3 operator-() const { return vec3(-x, -y, -z); }
	};

	BVH_DEVI vec3 operator+(const vec3& a, const vec3& b) { return vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
	BVH_DEVI vec3 operator-(const vec3& a, const vec3& b) { return vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
	BVH_DEVI vec3 operator*(const vec3& a, const vec3& b) { return vec3(a.x * b.x, a.y * b.y, a.z * b.z); }
	BVH_DEVI vec3 operator/(const vec3& a, const vec3& b) { return vec3(a.x / b.x, a.y / b.y, a.z / b.z); }

	BVH_DEVI vec3 operator*(const vec3& a, f32 s) { return vec3(a.x * s, a.y * s, a.z * s); }
	BVH_DEVI vec3 operator*(f32 s, const vec3& a) { return a * s; }
	BVH_DEVI vec3 operator/(const vec3& a, f32 s) { return vec3(a.x / s, a.y / s, a.z / s); }

	BVH_DEVI vec3& operator+=(vec3& a, const vec3& b) { a = a + b; return a; }
	BVH_DEVI vec3& operator-=(vec3& a, const vec3& b) { a = a - b; return a; }
	BVH_DEVI vec3& operator*=(vec3& a, const vec3& b) { a = a * b; return a; }
	BVH_DEVI vec3& operator*=(vec3& a, f32 s) { a = a * s; return a; }

	BVH_DEVI bool operator==(const vec3& a, const vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
	BVH_DEVI bool operator!=(const vec3& a, const vec3& b) { return !(a == b); }

	BVH_DEVI vec3 min(const vec3& a, const vec3& b)
	{
		return vec3(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z));
	}

	BVH_DEVI vec3 max(const vec3& a, const vec3& b)
	{
		return vec3(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z));
	}

	BVH_DEVI f32 dot(const vec3& a, const vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	
	BVH_DEVI vec3 cross(const vec3& a, const vec3& b)
	{
		return vec3( a.y * b.z - a.z * b.y,
					 a.z * b.x - a.x * b.z,
					 a.x * b.y - a.y * b.x);
	}

	BVH_DEVI f32 length_sq(const vec3& v) { return dot(v, v); }
	BVH_DEVI f32 length(const vec3& v) { return sqrtf(dot(v, v)); }
	BVH_DEVI vec3 normalize(const vec3 v) { return v / length(v); }
	BVH_DEVI vec3 abs(const vec3& v) { return vec3(abs(v.x), abs(v.y), abs(v.z)); }
	// component wise reciprocal, +/-inf makes an axis-parallel ray behave correctly in slab test.
	BVH_DEVI vec3 rcp(const vec3 v) { return vec3(1.0f / v.x, 1.0f / v.y, 1.0f / v.z); }

	// vec4

	struct alignas(16) vec4
	{
		union
		{
			f32 e[4];
			struct { f32 x, y, z, w; };
		};

		vec4() : e{ 0.0f, 0.0f, 0.0f, 0.0f } {}
		BVH_DEVI explicit vec4(f32 s) : e{ s,s,s,s } {}
		BVH_DEVI vec4(f32 x, f32 y, f32 z, f32 w) : e{ x,y,z,w } {}
		BVH_DEVI vec4(const vec3& v, f32 w) : e{ v.x, v.y, v.z, w } {}

		BVH_DEVI f32& operator[](u32 i) { return e[i]; }
		BVH_DEVI const f32& operator[](u32 i) const { return e[i]; }

		BVH_DEVI vec3 xyz() const { return vec3(x, y, z); }
	};

	// uvec3

	struct uvec3
	{
		union
		{
			u32 e[3];
			struct { u32 x, y, z; };
		};

		uvec3() : e{ 0u,0u,0u } {}
		BVH_DEVI uvec3(u32 x, u32 y, u32 z) : e{x,y,z} {}

		BVH_DEVI u32& operator[](u32 i) { return e[i]; }
		BVH_DEVI const u32& operator[](u32 i) const { return e[i]; }
	};

	static_assert(sizeof(vec2) == 8);
	static_assert(sizeof(vec3) == 12);
	static_assert(sizeof(vec4) == 16);
	static_assert(sizeof(uvec3) == 12);
} // namespace bvh

