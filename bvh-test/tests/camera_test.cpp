#include <util/camera.h>

#include <gtest/gtest.h>

#include <cmath>

using namespace bvh;

namespace {
constexpr u32 W = 64;
constexpr u32 H = 48;
} // namespace

TEST(Camera, AllRaysShareTheEyeOrigin)
{
	const vec3 eye(1.0f, 2.0f, 3.0f);
	const camera cam(W, H, 50.0f, eye, vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f));

	for (u32 j = 0; j < H; j += 7)
		for (u32 i = 0; i < W; i += 7)
		{
			const ray r = cam.generate_ray_through_pixel(i, j);
			EXPECT_TRUE(r.o == eye);
		}
}

TEST(Camera, DirectionIsNotNormalized)
{
	const camera cam = camera(W, H, 50.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));
	const ray corner = cam.generate_ray_through_pixel(0, 0);
	EXPECT_GT(length(corner.d), 1.0f);
}

TEST(Camera, CentreRayPointsAtTheTarget)
{
	const vec3 eye(0.0f, 0.0f, 5.0f);
	const vec3 target(0.0f, 0.0f, 0.0f);
	const camera cam(W, H, 50.0f, eye, target, vec3(0.0f, 1.0f, 0.0f));
	const ray a = cam.generate_ray_through_pixel(W / 2 - 1, H / 2 - 1);
	const ray b = cam.generate_ray_through_pixel(W / 2, H / 2);
	const vec3 avg = normalize((normalize(a.d) + normalize(b.d)) * 0.5f);
	const vec3 want = normalize(target - eye);

	EXPECT_NEAR(dot(avg, want), 1.0f, 1e-4f);
}

TEST(Camera, RowZeroIsTheBottomOfTheImage)
{
	const camera cam(W, H, 50.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));

	const ray bottom = cam.generate_ray_through_pixel(W / 2, 0);
	const ray top    = cam.generate_ray_through_pixel(W / 2, H - 1);

	EXPECT_LT(bottom.d.y, top.d.y);
}

TEST(Camera, HorizontalOrientationIsNotMirrored)
{
	const camera cam(W, H, 50.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));

	const ray left  = cam.generate_ray_through_pixel(0, H / 2);
	const ray right = cam.generate_ray_through_pixel(W - 1, H / 2);

	EXPECT_LT(left.d.x, right.d.x);
}

TEST(Camera, AspectRatioWidensHorizontalExtent)
{
	const camera square(64, 64, 50.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));
	const camera wide(128, 64, 50.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));
	const f32 sq = std::abs(square.generate_ray_through_pixel(0, 32).d.x);
	const f32 wd = std::abs(wide.generate_ray_through_pixel(0, 32).d.x);
	EXPECT_NEAR(wd / sq, 2.0f, 0.02f);
}

TEST(Camera, LongerFocalLengthNarrowsTheView)
{
	const camera wide_angle(W, H, 25.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));
	const camera telephoto(W, H, 100.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));

	EXPECT_GT(std::abs(wide_angle.generate_ray_through_pixel(0, H / 2).d.x),
	          std::abs(telephoto.generate_ray_through_pixel(0, H / 2).d.x));
}

TEST(Camera, FrameBoundsPlacesEyeOutsideTheScene)
{
	const aabb bounds(vec3(-1.0f, -1.0f, -1.0f), vec3(1.0f, 1.0f, 1.0f));
	const camera cam = camera::frame_bounds(bounds, W, H);

	const f32 radius = 0.5f * length(bounds.extent());
	const f32 dist   = length(cam.position() - bounds.centroid());

	EXPECT_GT(dist, radius);
}

TEST(Camera, FrameBoundsDistanceScalesWithSceneSize)
{
	const aabb small(vec3(-1.0f), vec3(1.0f));
	const aabb large(vec3(-10.0f), vec3(10.0f));

	const camera a = camera::frame_bounds(small, W, H);
	const camera b = camera::frame_bounds(large, W, H);

	const f32 da = length(a.position() - small.centroid());
	const f32 db = length(b.position() - large.centroid());

	EXPECT_NEAR(db / da, 10.0f, 1e-3f);
}

TEST(Camera, FrameBoundsDistanceMatchesTheFovFormula)
{
	const aabb bounds(vec3(-1.0f), vec3(1.0f));
	const f32  focal = 50.0f;
	const camera cam = camera::frame_bounds(bounds, W, H, focal);
	const f32 radius   = 0.5f * length(bounds.extent());
	const f32 expected = 1.25f * radius / (12.0f / focal);
	const f32 actual   = length(cam.position() - bounds.centroid());
	EXPECT_NEAR(actual, expected, 1e-3f);
}

TEST(Camera, FrameBoundsSurvivesDegenerateBounds)
{
	const aabb point(vec3(1.0f), vec3(1.0f));
	const camera cam = camera::frame_bounds(point, W, H);
	const ray r = cam.generate_ray_through_pixel(W / 2, H / 2);
	EXPECT_FALSE(std::isnan(r.d.x));
	EXPECT_FALSE(std::isnan(r.d.y));
	EXPECT_FALSE(std::isnan(r.d.z));
}

TEST(Camera, ReportsItsResolution)
{
	const camera cam(W, H, 50.0f, vec3(0.0f, 0.0f, 5.0f), vec3(0.0f), vec3(0.0f, 1.0f, 0.0f));
	EXPECT_EQ(cam.width(), W);
	EXPECT_EQ(cam.height(), H);
}
