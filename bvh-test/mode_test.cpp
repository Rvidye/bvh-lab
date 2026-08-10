#include <core/isect.h>
#include <core/mode.h>

#include <gtest/gtest.h>

#include <cmath>

using namespace bvh;

// opposite on isect_test.cpp default mode is robust so we fix alot of arches issues

namespace
{
	const aabb unit_box(vec3(-1.0f, -1.0f, -1.0f), vec3(1.0f, 1.0f, 1.0f));
	bool missed(f32 t, const ray& r) { return t >= r.t_max; }
} // namespace

TEST(RobustMode, IsTheDefault)
{
	static_assert(std::is_same_v<default_mode, robust_mode>);
	EXPECT_STREQ(default_mode::name, "robust");
	EXPECT_TRUE(robust_mode::slab_padding);
	EXPECT_TRUE(robust_mode::reject_degenerate);
	EXPECT_FALSE(arches_mode::slab_padding);
	EXPECT_FALSE(arches_mode::reject_degenerate);
}

// degenerate triangles

TEST(RobustMode, EdgeOnTriangleIsRejected)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	const ray r(vec3(-1.0f, 0.25f, 0.0f), vec3(1.0f, 0.0f, 0.0f)); // coplanar

	hit h;
	EXPECT_FALSE(intersect<robust_mode>(tri, r, h));
	EXPECT_FALSE(h.valid());
	EXPECT_FALSE(std::isnan(h.t));

	// The same call under arches_mode is accepted with a NaN distance.
	hit ah;
	EXPECT_TRUE(intersect<arches_mode>(tri, r, ah));
	EXPECT_TRUE(std::isnan(ah.t));
}

TEST(RobustMode, ZeroAreaTriangleIsRejected)
{
	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(2.0f, 0.0f, 0.0f));
	const ray r(vec3(0.5f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	EXPECT_FALSE(intersect<robust_mode>(tri, r, h));
	EXPECT_FALSE(std::isnan(h.t));
}

TEST(RobustMode, NaNNeverEntersTheHitRecord)
{
	const triangle edge_on(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));
	const triangle behind(vec3(-10.0f, 0.0f, -1.0f), vec3(10.0f, 0.0f, -1.0f), vec3(0.0f, 10.0f, -1.0f));
	const ray r(vec3(-1.0f, 0.25f, 0.0f), vec3(1.0f, 0.0f, 0.0f));

	hit h;
	EXPECT_FALSE(intersect<robust_mode>(edge_on, r, h));
	ASSERT_FALSE(std::isnan(h.t));

	// h.t is still t_max, so the closest-hit guard is intact and a triangle
	// nowhere near the ray is still rejected.
	EXPECT_FALSE(intersect<robust_mode>(behind, r, h));
	EXPECT_FALSE(h.valid());
}

TEST(RobustMode, WellFormedTrianglesStillHit)
{
	for (f32 scale : {0.01f, 0.1f, 1.0f, 10.0f, 100.0f})
	{
		const triangle tri(vec3(0.0f, 0.0f, 0.0f),
			vec3(scale, 0.0f, 0.0f),
			vec3(0.0f, scale, 0.0f));
		const ray r(vec3(scale * 0.25f, scale * 0.25f, -scale), vec3(0.0f, 0.0f, 1.0f), scale * 1e-3f, scale * 1e3f);

		hit h;
		h.t = r.t_max;

		EXPECT_TRUE(intersect<robust_mode>(tri, r, h)) << "scale " << scale;
		EXPECT_NEAR(h.t, scale, scale * 1e-4f) << "scale " << scale;
	}
}

TEST(RobustMode, DefaultTMinRejectsHitsInsideTinyGeometry)
{
	constexpr f32 scale = 0.01f; // whole triangle is smaller than t_min = 1/32

	const triangle tri(vec3(0.0f, 0.0f, 0.0f), vec3(scale, 0.0f, 0.0f), vec3(0.0f, scale, 0.0f));
	const ray near_ray(vec3(scale * 0.25f, scale * 0.25f, -scale), vec3(0.0f, 0.0f, 1.0f));

	ASSERT_GT(t_min_default, scale) << "premise: t_min is larger than the model";

	hit h;
	EXPECT_FALSE(intersect<robust_mode>(tri, near_ray, h)) << "hit at t=" << scale << " is inside the default t_min of " << t_min_default;

	// Same geometry, explicit small t_min: now it hits.
	const ray tuned(near_ray.o, near_ray.d, scale * 1e-3f, scale * 1e3f);
	hit h2;
	h2.t = tuned.t_max;
	EXPECT_TRUE(intersect<robust_mode>(tri, tuned, h2));
}

TEST(RobustMode, GrazingRayOnMaxPlaneNowHits)
{
	// Arches misses this and hits the mirror case on the min plane -- an
	// asymmetry caused purely by min/max operand order. Ize padding removes it.
	const ray r(vec3(-5.0f, 1.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f)); // y exactly on max.y

	const f32 robust = intersect<robust_mode>(unit_box, r, rcp(r.d));
	const f32 arches = intersect<arches_mode>(unit_box, r, rcp(r.d));

	EXPECT_FALSE(std::isnan(robust));
	EXPECT_TRUE(missed(arches, r)) << "arches parity: the max plane misses";
	EXPECT_FALSE(missed(robust, r)) << "robust mode should not lose a grazing hit";
	EXPECT_FLOAT_EQ(robust, 4.0f);
}

TEST(RobustMode, GrazingIsSymmetricAcrossMinAndMaxPlanes)
{
	const ray on_min(vec3(-5.0f, -1.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	const ray on_max(vec3(-5.0f, 1.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));

	const f32 a = intersect<robust_mode>(unit_box, on_min, rcp(on_min.d));
	const f32 b = intersect<robust_mode>(unit_box, on_max, rcp(on_max.d));

	EXPECT_FLOAT_EQ(a, b) << "min and max plane grazing must agree in robust mode";
}

TEST(RobustMode, PaddingDoesNotTurnClearMissesIntoHits)
{
	// The padding is 1 + 2*gamma(3) ~ 1.00000024, so it must not widen the box
	// enough to catch a ray that genuinely passes outside it.
	const ray r(vec3(-5.0f, 5.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	EXPECT_TRUE(missed(intersect<robust_mode>(unit_box, r, rcp(r.d)), r));

	const ray behind(vec3(5.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	EXPECT_TRUE(missed(intersect<robust_mode>(unit_box, behind, rcp(behind.d)), behind));
}

TEST(RobustMode, OrdinaryHitsAreUnchangedByPadding)
{
	// Padding must be invisible everywhere except at the boundary; if it moved
	// ordinary entry distances it would change every reported t.
	const ray r(vec3(-5.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f));

	EXPECT_FLOAT_EQ(intersect<robust_mode>(unit_box, r, rcp(r.d)), 4.0f);
	EXPECT_FLOAT_EQ(intersect<arches_mode>(unit_box, r, rcp(r.d)), 4.0f);
}
