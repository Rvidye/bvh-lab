#include <core/isect.h>
#include <core/mode.h>

#include <gtest/gtest.h>

#include <cmath>

using namespace bvh;

using M = arches_mode;

namespace {
const aabb unit_box(vec3(-1.0f, -1.0f, -1.0f), vec3(1.0f, 1.0f, 1.0f));

f32 hit_box(const aabb& box, const vec3& o, const vec3& d)
{
	const ray r(o, d);
	return intersect<M>(box, r, rcp(d));
}

bool missed(f32 t, const ray& r) { return t >= r.t_max; }

} // namespace


TEST(IntersectAABB, HitFromOutsideReturnsEntryDistance)
{
	const f32 t = hit_box(unit_box, vec3(-5.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	EXPECT_FLOAT_EQ(t, 4.0f);
}

TEST(IntersectAABB, MissReturnsRayTMax)
{
	const ray r(vec3(-5.0f, 5.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	const f32 t = intersect<M>(unit_box, r, rcp(r.d));
	EXPECT_TRUE(missed(t, r));
}

TEST(IntersectAABB, BoxBehindRayMisses)
{
	const ray r(vec3(5.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	const f32 t = intersect<M>(unit_box, r, rcp(r.d));
	EXPECT_TRUE(missed(t, r));
}

TEST(IntersectAABB, DegenerateBoxMisses)
{
	aabb degenerate;
	const ray r(vec3(0.0f, 0.0f, -5.0f), vec3(0.0f, 0.0f, 1.0f));
	const f32 t = intersect<M>(degenerate, r, rcp(r.d));
	EXPECT_TRUE(missed(t, r));
}

TEST(IntersectAABB, AxisParallelRayThroughInteriorHits)
{
	const ray r(vec3(-5.0f, 0.5f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	const f32 t = intersect<M>(unit_box, r, rcp(r.d));

	EXPECT_FALSE(std::isnan(t));
	EXPECT_FLOAT_EQ(t, 4.0f);
}

TEST(IntersectAABB, GrazingRayOnMinPlaneHits)
{
	const ray r(vec3(-5.0f, -1.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f)); // y on min.y
	const f32 t = intersect<M>(unit_box, r, rcp(r.d));

	EXPECT_FALSE(std::isnan(t));
	EXPECT_FLOAT_EQ(t, 4.0f);
}

TEST(IntersectAABB, GrazingRayOnMaxPlaneMissesLikeArches)
{
	const ray r(vec3(-5.0f, 1.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f)); // y on max.y
	const f32 t = intersect<M>(unit_box, r, rcp(r.d));

	EXPECT_FALSE(std::isnan(t)) << "NaN must not escape the box test";
	EXPECT_TRUE(missed(t, r)) << "asymmetric with the min plane; matches Arches";
}

TEST(IntersectAABB, RayStartingInsideReturnsTMin)
{
	const ray r(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	const f32 t = intersect<M>(unit_box, r, rcp(r.d));
	EXPECT_FLOAT_EQ(t, r.t_min);
}

TEST(IntersectAABB, RespectsRayTMin)
{
	const aabb tiny(vec3(-0.001f, -0.001f, -0.001f), vec3(0.001f, 0.001f, 0.001f));
	const ray  r(vec3(0.0f, 0.0f, -0.002f), vec3(0.0f, 0.0f, 1.0f));
	const f32  t = intersect<M>(tiny, r, rcp(r.d));
	EXPECT_TRUE(missed(t, r));
}

TEST(IntersectTriangle, CentreHit)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	const ray r(vec3(0.25f, 0.25f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	ASSERT_TRUE(intersect<M>(tri, r, h));
	EXPECT_FLOAT_EQ(h.t, 1.0f);
}

TEST(IntersectTriangle, BarycentricXWeightsVertexZero)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	hit h;
	const ray r(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	ASSERT_TRUE(intersect<M>(tri, r, h));
	EXPECT_NEAR(h.bc.x, 1.0f, 1e-5f);
	EXPECT_NEAR(h.bc.y, 0.0f, 1e-5f);
}

TEST(IntersectTriangle, BarycentricYWeightsVertexOne)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));

	hit h;
	const ray r(vec3(1.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	ASSERT_TRUE(intersect<M>(tri, r, h));
	EXPECT_NEAR(h.bc.x, 0.0f, 1e-5f);
	EXPECT_NEAR(h.bc.y, 1.0f, 1e-5f);
}

TEST(IntersectTriangle, RemainderWeightsVertexTwo)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));

	hit h;
	const ray r(vec3(0.0f, 1.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	ASSERT_TRUE(intersect<M>(tri, r, h));
	EXPECT_NEAR(1.0f - h.bc.x - h.bc.y, 1.0f, 1e-5f);
}

TEST(IntersectTriangle, MissesOutsideEdges)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));

	hit h;
	const ray r(vec3(0.9f, 0.9f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_FALSE(intersect<M>(tri, r, h));
	EXPECT_FALSE(h.valid());
}

TEST(IntersectTriangle, MissesBehindOrigin)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	hit h;
	const ray r(vec3(0.25f, 0.25f, 1.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_FALSE(intersect<M>(tri, r, h));
}

TEST(IntersectTriangle, HitsAreTwoSided)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	hit h;
	const ray r(vec3(0.25f, 0.25f, 1.0f), vec3(0.0f, 0.0f, -1.0f));
	EXPECT_TRUE(intersect<M>(tri, r, h));
	EXPECT_FLOAT_EQ(h.t, 1.0f);
}

TEST(IntersectTriangle, KeepsNearestOnRepeatedCalls)
{
	const triangle near_tri(vec3(0.0f, 0.0f, 1.0f), vec3(1.0f, 0.0f, 1.0f), vec3(0.0f, 1.0f, 1.0f));
	const triangle far_tri(vec3(0.0f, 0.0f, 5.0f), vec3(1.0f, 0.0f, 5.0f), vec3(0.0f, 1.0f, 5.0f));
	const ray r(vec3(0.25f, 0.25f, 0.0f), vec3(0.0f, 0.0f, 1.0f));
	hit h;
	EXPECT_TRUE(intersect<M>(far_tri, r, h));
	EXPECT_FLOAT_EQ(h.t, 5.0f);
	EXPECT_TRUE(intersect<M>(near_tri, r, h));
	EXPECT_FLOAT_EQ(h.t, 1.0f);
	EXPECT_FALSE(intersect<M>(far_tri, r, h));
	EXPECT_FLOAT_EQ(h.t, 1.0f);
}

TEST(IntersectTriangle, DegenerateTriangleIsNotRejectedByTheIntersector)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(2.0f, 0.0f, 0.0f));
	const ray r(vec3(0.5f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	const bool accepted = intersect<M>(tri, r, h);

	// Documenting current behaviour. If this ever starts returning false,
	// something added a determinant guard and we have diverged from Arches.
	EXPECT_TRUE(accepted) << "rtm has no determinant guard; divergence from Arches";
}

TEST(IntersectTriangle, EdgeOnTriangleIsAcceptedWithNaN)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	const ray r(vec3(-1.0f, 0.25f, 0.0f), vec3(1.0f, 0.0f, 0.0f)); // coplanar

	hit h;
	const bool accepted = intersect<M>(tri, r, h);

	EXPECT_TRUE(accepted) << "matches Arches; see comment above";
	EXPECT_TRUE(std::isnan(h.t)) << "and the recorded distance is NaN";
}

TEST(IntersectTriangle, NaNHitPoisonsSubsequentClosestHitTests)
{
	const triangle edge_on(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	const triangle normal_tri(vec3(-10.0f, 0.0f, -1.0f), vec3(10.0f, 0.0f, -1.0f), vec3(0.0f, 10.0f, -1.0f));

	const ray r(vec3(-1.0f, 0.25f, 0.0f), vec3(1.0f, 0.0f, 0.0f));

	hit h;
	ASSERT_TRUE(intersect<M>(edge_on, r, h));
	ASSERT_TRUE(std::isnan(h.t));

	// h.t is NaN, so the `t > h.t` rejection can no longer reject anything.
	// A triangle nowhere near the ray is now free to overwrite the record.
	const bool poisoned = intersect<M>(normal_tri, r, h);
	EXPECT_FALSE(poisoned) << "if this ever passes, degenerate filtering became mandatory";
}
