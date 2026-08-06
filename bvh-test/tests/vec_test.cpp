#include <core/vec.h>

#include <gtest/gtest.h>

#include <cmath>

using namespace bvh;

TEST(Vec3, DivideByScalarDividesEveryComponent)
{
	const vec3 v(2.0f, 4.0f, 8.0f);
	const vec3 r = v / 2.0f;

	EXPECT_FLOAT_EQ(r.x, 1.0f);
	EXPECT_FLOAT_EQ(r.y, 2.0f);
	EXPECT_FLOAT_EQ(r.z, 4.0f);
}

TEST(Vec3, NormalizeProducesUnitLength)
{
	const vec3 cases[] = {
		vec3(1.0f, 2.0f, 3.0f),
		vec3(-0.6f, -0.35f, -1.0f), // the default frame_bounds direction
		vec3(0.0f, 0.0f, 5.0f),
		vec3(-3.0f, 0.0f, 0.0f),
	};

	for (const vec3& v : cases)
	{
		const vec3 n = normalize(v);
		EXPECT_NEAR(length(n), 1.0f, 1e-6f) << "v = (" << v.x << ", " << v.y << ", " << v.z << ")";
	}
}

TEST(Vec3, NormalizePreservesDirection)
{
	const vec3 v(1.0f, 2.0f, 3.0f);
	const vec3 n = normalize(v);
	const f32  len = length(v);

	EXPECT_FLOAT_EQ(n.x, v.x / len);
	EXPECT_FLOAT_EQ(n.y, v.y / len);
	EXPECT_FLOAT_EQ(n.z, v.z / len);
}

TEST(Scalar, UnsignedMinReturnsSmaller)
{
	EXPECT_EQ(bvh::min(3u, 7u), 3u);
	EXPECT_EQ(bvh::min(7u, 3u), 3u);
	EXPECT_EQ(bvh::min(5u, 5u), 5u);
	EXPECT_EQ(bvh::min(0u, ~0u), 0u);
}

TEST(Scalar, UnsignedMaxReturnsLarger)
{
	EXPECT_EQ(bvh::max(3u, 7u), 7u);
	EXPECT_EQ(bvh::max(7u, 3u), 7u);
	EXPECT_EQ(bvh::max(0u, ~0u), ~0u);
}

TEST(Scalar, FloatMinMax)
{
	EXPECT_FLOAT_EQ(bvh::min(-2.0f, 3.0f), -2.0f);
	EXPECT_FLOAT_EQ(bvh::max(-2.0f, 3.0f), 3.0f);
	EXPECT_FLOAT_EQ(bvh::min(3.0f, -2.0f), -2.0f);
	EXPECT_FLOAT_EQ(bvh::max(3.0f, -2.0f), 3.0f);
}

TEST(Scalar, MinMaxNaNMatchesArches)
{
	const f32 nan = std::nanf("");
	ASSERT_TRUE(std::isnan(nan));
	EXPECT_FLOAT_EQ(bvh::min(1.0f, nan), 1.0f);
	EXPECT_FLOAT_EQ(bvh::max(1.0f, nan), 1.0f);
	EXPECT_TRUE(std::isnan(bvh::min(nan, 1.0f)));
	EXPECT_TRUE(std::isnan(bvh::max(nan, 1.0f)));
}

TEST(Vec3, Arithmetic)
{
	const vec3 a(1.0f, 2.0f, 3.0f);
	const vec3 b(4.0f, 5.0f, 6.0f);

	EXPECT_TRUE(a + b == vec3(5.0f, 7.0f, 9.0f));
	EXPECT_TRUE(b - a == vec3(3.0f, 3.0f, 3.0f));
	EXPECT_TRUE(a * b == vec3(4.0f, 10.0f, 18.0f));
	EXPECT_TRUE(a * 2.0f == vec3(2.0f, 4.0f, 6.0f));
	EXPECT_TRUE(2.0f * a == vec3(2.0f, 4.0f, 6.0f));
	EXPECT_TRUE(-a == vec3(-1.0f, -2.0f, -3.0f));
}

TEST(Vec3, DotAndCross)
{
	const vec3 x(1.0f, 0.0f, 0.0f);
	const vec3 y(0.0f, 1.0f, 0.0f);
	const vec3 z(0.0f, 0.0f, 1.0f);

	EXPECT_FLOAT_EQ(dot(x, y), 0.0f);
	EXPECT_FLOAT_EQ(dot(x, x), 1.0f);
	EXPECT_FLOAT_EQ(dot(vec3(1.0f, 2.0f, 3.0f), vec3(4.0f, 5.0f, 6.0f)), 32.0f);

	// Right-handed: x cross y == z.
	EXPECT_TRUE(cross(x, y) == z);
	EXPECT_TRUE(cross(y, z) == x);
	EXPECT_TRUE(cross(z, x) == y);
	// Anticommutative.
	EXPECT_TRUE(cross(y, x) == -z);
}

TEST(Vec3, ComponentIndexingMatchesNamedMembers)
{
	vec3 v(1.0f, 2.0f, 3.0f);
	EXPECT_FLOAT_EQ(v[0], v.x);
	EXPECT_FLOAT_EQ(v[1], v.y);
	EXPECT_FLOAT_EQ(v[2], v.z);

	v[2] = 9.0f;
	EXPECT_FLOAT_EQ(v.z, 9.0f);
}

TEST(Vec3, ReciprocalYieldsInfinityOnZero)
{
	const vec3 r = rcp(vec3(2.0f, 0.0f, -0.0f));
	EXPECT_FLOAT_EQ(r.x, 0.5f);
	EXPECT_TRUE(std::isinf(r.y));
	EXPECT_GT(r.y, 0.0f);
	EXPECT_TRUE(std::isinf(r.z));
	EXPECT_LT(r.z, 0.0f);
}

TEST(Vec, LayoutIsPodAndPacked)
{
	EXPECT_EQ(sizeof(vec2), 8u);
	EXPECT_EQ(sizeof(vec3), 12u);
	EXPECT_EQ(sizeof(vec4), 16u);
	EXPECT_EQ(sizeof(uvec3), 12u);
	EXPECT_EQ(alignof(vec4), 16u);
}
