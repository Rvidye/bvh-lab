#include <core/ray.h>

#include <gtest/gtest.h>

using namespace bvh;

TEST(Ray, LayoutMatchesArches)
{
	EXPECT_EQ(sizeof(ray), 32u);
	EXPECT_EQ(sizeof(hit), 16u);

	EXPECT_EQ(offsetof(ray, o), 0u);
	EXPECT_EQ(offsetof(ray, t_min), 12u);
	EXPECT_EQ(offsetof(ray, d), 16u);
	EXPECT_EQ(offsetof(ray, t_max), 28u);

	EXPECT_EQ(offsetof(hit, t), 0u);
	EXPECT_EQ(offsetof(hit, bc), 4u);
	EXPECT_EQ(offsetof(hit, id), 12u);
}

TEST(Ray, DefaultRangeMatchesArchesConstants)
{
	EXPECT_FLOAT_EQ(t_max_default, 1048576.0f);
	EXPECT_FLOAT_EQ(t_min_default, 0.03125f);

	const ray r(vec3(0.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_FLOAT_EQ(r.t_min, t_min_default);
	EXPECT_FLOAT_EQ(r.t_max, t_max_default);
}

TEST(Hit, DefaultsToInvalid)
{
	const hit h;
	EXPECT_FALSE(h.valid());
	EXPECT_EQ(h.id, invalid_id);
	EXPECT_FLOAT_EQ(h.t, t_max_default);
}

TEST(Triangle, Bounds)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 0.0f, 1.0f), vec3(1.0f, 3.0f, -1.0f));
	const aabb b = tri.bounds();

	EXPECT_TRUE(b.min == vec3(0.0f, 0.0f, -1.0f));
	EXPECT_TRUE(b.max == vec3(2.0f, 3.0f, 1.0f));
}

TEST(Triangle, Centroid)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(3.0f, 0.0f, 0.0f), vec3(0.0f, 3.0f, 0.0f));
	EXPECT_TRUE(tri.centroid() == vec3(1.0f, 1.0f, 0.0f));
}

TEST(Triangle, NormalIsNormalized)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(4.0f, 0.0f, 0.0f), vec3(0.0f, 7.0f, 0.0f));
	const vec3 n = tri.normal();

	EXPECT_NEAR(length(n), 1.0f, 1e-6f);
	EXPECT_NEAR(std::abs(n.z), 1.0f, 1e-6f);
	EXPECT_NEAR(n.x, 0.0f, 1e-6f);
	EXPECT_NEAR(n.y, 0.0f, 1e-6f);
}

TEST(Triangle, LayoutMatchesArches)
{
	EXPECT_EQ(alignof(triangle), 64u);
}
