#include <reference/brute_force.h>
#include <core/rng.h>
#include <util/camera.h>
#include <gtest/gtest.h>
#include <filesystem>
#include <string>

using namespace bvh;

namespace {

std::string find_scene(const char* name)
{
	const char* roots[] = {"scenes/", "../scenes/", "../../scenes/", "../../../scenes/"};
	for (const char* root : roots)
	{
		const std::string p = std::string(root) + name;
		if (std::filesystem::exists(p)) return p;
	}
	return {};
}

mesh two_quads()
{
	mesh m;
	m.vertices = {
	    // near quad at z = 1
	    vec3(-1.0f, -1.0f, 1.0f), vec3(1.0f, -1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f), vec3(-1.0f, 1.0f, 1.0f),
	    // far quad at z = 5
	    vec3(-1.0f, -1.0f, 5.0f), vec3(1.0f, -1.0f, 5.0f), vec3(1.0f, 1.0f, 5.0f), vec3(-1.0f, 1.0f, 5.0f),
	};
	m.vertex_indices = {
	    uvec3(0, 1, 2), uvec3(0, 2, 3),
	    uvec3(4, 5, 6), uvec3(4, 6, 7),
	};
	m.normal_indices.assign(4, uvec3(invalid_id, invalid_id, invalid_id));
	m.tex_coord_indices.assign(4, uvec3(invalid_id, invalid_id, invalid_id));
	m.material_indices.assign(4, invalid_id);
	m.compute_bounds();
	return m;
}

} // namespace

TEST(BruteForce, FindsTheNearestHit)
{
	const mesh m = two_quads();
	const ray r(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	null_stats stats;
	ASSERT_TRUE(intersect_brute_force(m, r, h, stats));

	EXPECT_FLOAT_EQ(h.t, 2.0f); // near quad at z = 1, origin at z = -1
	EXPECT_TRUE(h.id == 0u || h.id == 1u) << "expected a near-quad triangle";
}

TEST(BruteForce, OrderIndependence)
{
	mesh forward = two_quads();
	mesh reversed = two_quads();
	reversed.reorder({3, 2, 1, 0});

	const ray r(vec3(0.1f, -0.2f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit a, b;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(forward, r, a, s));
	ASSERT_TRUE(intersect_brute_force(reversed, r, b, s));

	EXPECT_FLOAT_EQ(a.t, b.t);
}

TEST(BruteForce, MissLeavesHitInvalid)
{
	const mesh m = two_quads();
	const ray r(vec3(10.0f, 10.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	null_stats stats;
	EXPECT_FALSE(intersect_brute_force(m, r, h, stats));
	EXPECT_FALSE(h.valid());
	EXPECT_FLOAT_EQ(h.t, t_max_default);
}

TEST(BruteForce, StatsUseArchesStepSemantics)
{
	const mesh m = two_quads();
	const ray r(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	trace_stats stats;
	intersect_brute_force(m, r, h, stats);

	EXPECT_EQ(stats.prim_steps, 1u);
	EXPECT_EQ(stats.tri_tests, m.triangle_count());
	EXPECT_EQ(stats.node_steps, 0u) << "brute force visits no interior nodes";
	EXPECT_EQ(stats.total_steps(), 1u);
}

TEST(BruteForce, NullStatsCompileAndCostNothing)
{
	const mesh m = two_quads();
	const ray r(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h;
	null_stats stats;
	EXPECT_TRUE(intersect_brute_force(m, r, h, stats));
	EXPECT_TRUE(intersect_brute_force(m, r, h));
}

TEST(BruteForce, OccludedAgreesWithClosestHit)
{
	const mesh m = two_quads();
	null_stats stats;

	const ray blocked(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_TRUE(occluded_brute_force(m, blocked, stats));

	const ray clear(vec3(10.0f, 10.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_FALSE(occluded_brute_force(m, clear, stats));
}

TEST(BruteForce, RendersAScene)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const u32 res = 64;
	const camera cam = camera::frame_bounds(m.bounds(), res, res);
	image img(res, res);

	const render_result r = render_normals(m, cam, img, 2);

	EXPECT_EQ(r.rays, u64(res) * res);
	EXPECT_GT(r.hits, 0u) << "the teapot should be visible";
	EXPECT_LT(r.hits, r.rays) << "the teapot should not fill the frame";

	const double hit_rate = double(r.hits) / double(r.rays);
	EXPECT_GT(hit_rate, 0.02);
	EXPECT_LT(hit_rate, 0.50) << "hit rate this high means the camera is too close";
}

TEST(BruteForce, RenderIsDeterministic)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const u32 res = 32;
	const camera cam = camera::frame_bounds(m.bounds(), res, res);

	image a(res, res), b(res, res);
	const render_result ra = render_normals(m, cam, a, 1);
	const render_result rb = render_normals(m, cam, b, 4);

	EXPECT_EQ(ra.hits, rb.hits);
	for (u32 y = 0; y < res; ++y)
		for (u32 x = 0; x < res; ++x)
		{
			const vec3 pa = a.get(x, y);
			const vec3 pb = b.get(x, y);
			ASSERT_FLOAT_EQ(pa.x, pb.x) << "pixel (" << x << "," << y << ")";
			ASSERT_FLOAT_EQ(pa.y, pb.y);
			ASSERT_FLOAT_EQ(pa.z, pb.z);
		}
}
