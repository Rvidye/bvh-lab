#include <core/aabb.h>

#include <gtest/gtest.h>

using namespace bvh;

TEST(AABB, SurfaceAreaOfUnitCubeIsSix)
{
	const aabb box(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 1.0f));
	EXPECT_FLOAT_EQ(box.surface_area(), 6.0f);
}

TEST(AABB, SurfaceAreaUsesAllThreeFacePairs)
{
	const aabb box(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 3.0f, 4.0f));
	EXPECT_FLOAT_EQ(box.surface_area(), 52.0f);
}

TEST(AABB, SurfaceAreaIsTranslationInvariant)
{
	const aabb a(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 3.0f, 4.0f));
	const aabb b(vec3(-5.0f, 7.0f, -1.0f), vec3(-3.0f, 10.0f, 3.0f));
	EXPECT_FLOAT_EQ(a.surface_area(), b.surface_area());
}

TEST(AABB, SurfaceAreaOfFlatBoxIsTwoFaces)
{
	const aabb box(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 3.0f, 0.0f));
	EXPECT_FLOAT_EQ(box.surface_area(), 12.0f);
}

TEST(AABB, DefaultConstructedIsEmptyWithZeroArea)
{
	const aabb box;
	EXPECT_TRUE(box.empty());
	EXPECT_FLOAT_EQ(box.surface_area(), 0.0f);
}

TEST(AABB, AddPointGrowsBounds)
{
	aabb box;
	box.add(vec3(1.0f, 2.0f, 3.0f));

	EXPECT_FALSE(box.empty());
	EXPECT_TRUE(box.min == vec3(1.0f, 2.0f, 3.0f));
	EXPECT_TRUE(box.max == vec3(1.0f, 2.0f, 3.0f));

	box.add(vec3(-1.0f, 5.0f, 0.0f));
	EXPECT_TRUE(box.min == vec3(-1.0f, 2.0f, 0.0f));
	EXPECT_TRUE(box.max == vec3(1.0f, 5.0f, 3.0f));
}

TEST(AABB, AddEmptyBoxIsANoOp)
{
	aabb box(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 1.0f));
	const aabb before = box;

	box.add(aabb{});

	EXPECT_TRUE(box.min == before.min);
	EXPECT_TRUE(box.max == before.max);
	EXPECT_FLOAT_EQ(box.surface_area(), 6.0f);
}

TEST(AABB, Centroid)
{
	const aabb box(vec3(-2.0f, 0.0f, 4.0f), vec3(2.0f, 4.0f, 6.0f));
	EXPECT_TRUE(box.centroid() == vec3(0.0f, 2.0f, 5.0f));
}

TEST(AABB, LongestAxis)
{
	EXPECT_EQ(aabb(vec3(0.0f), vec3(5.0f, 1.0f, 2.0f)).longest_axis(), 0u);
	EXPECT_EQ(aabb(vec3(0.0f), vec3(1.0f, 5.0f, 2.0f)).longest_axis(), 1u);
	EXPECT_EQ(aabb(vec3(0.0f), vec3(1.0f, 2.0f, 5.0f)).longest_axis(), 2u);
	EXPECT_EQ(aabb(vec3(0.0f), vec3(3.0f, 3.0f, 3.0f)).longest_axis(), 0u);
	EXPECT_EQ(aabb(vec3(0.0f), vec3(1.0f, 3.0f, 3.0f)).longest_axis(), 1u);
}

TEST(AABB, Merge)
{
	const aabb a(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 1.0f));
	const aabb b(vec3(2.0f, -1.0f, 0.5f), vec3(3.0f, 0.0f, 4.0f));
	const aabb m = merge(a, b);

	EXPECT_TRUE(m.min == vec3(0.0f, -1.0f, 0.0f));
	EXPECT_TRUE(m.max == vec3(3.0f, 1.0f, 4.0f));
}

TEST(AABB, LayoutIsPacked)
{
	EXPECT_EQ(sizeof(aabb), 24u);
}
