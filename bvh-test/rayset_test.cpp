#include <build/bvh2_builder.h>
#include <core/sampling.h>
#include <eval/rayset.h>
#include <eval/trace.h>
#include <util/camera.h>

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>

using namespace bvh;

namespace {

	std::string find_scene(const char* name)
	{
		const char* roots[] = { "scenes/", "../scenes/", "../../scenes/", "../../../scenes/" };
		for (const char* root : roots)
		{
			const std::string p = std::string(root) + name;
			if (std::filesystem::exists(p)) return p;
		}
		return {};
	}

	struct fixture
	{
		mesh   m;
		bvh2   tree;
		camera cam;
		bool   ok{ false };

		fixture(u32 res = 64)
		{
			const std::string p = find_scene("teapot.obj");
			if (p.empty() || !m.load_obj(p)) return;

			build_args a;
			a.silent = true;
			tree.build(m, a);
			tree.apply_reorder(m);

			cam = camera::frame_bounds(m.bounds(), res, res);
			ok = true;
		}
	};

	rayset_args small_args(u32 res = 64)
	{
		rayset_args a;
		a.width = a.height = res;
		a.incoherent_count = 4096;
		return a;
	}

	std::string temp_path(const char* stem)
	{
		const std::filesystem::path p =
			std::filesystem::temp_directory_path() / (std::string("bvhlab_") + stem + ".rays");
		std::filesystem::remove(p);
		return p.string();
	}

} // namespace

// sampling primitives
TEST(Sampling, OrthonormalBasisIsOrthonormalEverywhere)
{
	const vec3 normals[] = {
		vec3(0.0f, 0.0f, 1.0f),  vec3(0.0f, 0.0f, -1.0f),
		vec3(1.0f, 0.0f, 0.0f),  vec3(0.0f, 1.0f, 0.0f),
		normalize(vec3(1.0f, 1.0f, 1.0f)), normalize(vec3(-0.3f, 0.9f, -0.2f)),
		vec3(0.0f, 0.0f, 0.99999994f),
	};

	for (const vec3& n : normals)
	{
		vec3 t, b;
		orthonormal_basis(n, t, b);

		EXPECT_NEAR(length(t), 1.0f, 1e-4f);
		EXPECT_NEAR(length(b), 1.0f, 1e-4f);
		EXPECT_NEAR(dot(t, b), 0.0f, 1e-4f);
		EXPECT_NEAR(dot(t, n), 0.0f, 1e-4f);
		EXPECT_NEAR(dot(b, n), 0.0f, 1e-4f);
	}
}

TEST(Sampling, CosineHemisphereStaysInTheHemisphere)
{
	const vec3 n = normalize(vec3(0.2f, 0.9f, -0.3f));
	rng random(1234u, 1u);

	for (int i = 0; i < 5000; ++i)
	{
		const vec3 v = cosine_hemisphere(n, random.next_f32(), random.next_f32());
		ASSERT_NEAR(length(v), 1.0f, 1e-3f);
		ASSERT_GE(dot(v, n), -1e-4f) << "sample fell below the hemisphere";
	}
}

TEST(Sampling, CosineHemisphereIsCosineWeighted)
{
	const vec3 n(0.0f, 0.0f, 1.0f);
	rng random(99u, 1u);

	double sum = 0.0;
	const int N = 200000;
	for (int i = 0; i < N; ++i)
		sum += dot(cosine_hemisphere(n, random.next_f32(), random.next_f32()), n);

	EXPECT_NEAR(sum / N, 2.0 / 3.0, 0.01);
}

TEST(Sampling, UniformSphereIsUnitAndBalanced)
{
	rng random(7u, 1u);
	vec3 mean(0.0f);
	const int N = 100000;

	for (int i = 0; i < N; ++i)
	{
		const vec3 v = uniform_sphere(random.next_f32(), random.next_f32());
		ASSERT_NEAR(length(v), 1.0f, 1e-4f);
		mean += v;
	}

	EXPECT_LT(length(mean * (1.0f / N)), 0.02f) << "directions should cancel out";
}

// generation
TEST(RaySet, GeneratesEveryDistribution)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	for (ray_distribution dist : all_ray_distributions)
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, small_args());
		EXPECT_GT(rs.size(), 0u) << to_string(dist);
		EXPECT_EQ(rs.distribution, dist);
		EXPECT_EQ(rs.o.size(), rs.d.size());
		EXPECT_EQ(rs.o.size(), rs.t_min.size());
		EXPECT_EQ(rs.o.size(), rs.t_max.size());
	}
}

TEST(RaySet, PrimaryCoversEveryPixel)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset rs = rayset::generate(ray_distribution::primary, f.m, f.tree, f.cam, small_args());
	EXPECT_EQ(rs.size(), 64u * 64u);
}

TEST(RaySet, SecondarySetsAreShorterThanPrimary)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const u32 pixels = 64u * 64u;
	for (ray_distribution dist : {ray_distribution::shadow_ao, ray_distribution::reflection,
		ray_distribution::diffuse_1, ray_distribution::diffuse_n})
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, small_args());
		EXPECT_GT(rs.size(), 0u) << to_string(dist);
		EXPECT_LT(rs.size(), pixels) << to_string(dist) << " should drop primary misses";
	}
}

TEST(RaySet, DiffuseNIsShorterThanDiffuse1)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset one = rayset::generate(ray_distribution::diffuse_1, f.m, f.tree, f.cam, small_args());
	const rayset n = rayset::generate(ray_distribution::diffuse_n, f.m, f.tree, f.cam, small_args());

	EXPECT_LT(n.size(), one.size());
}

TEST(RaySet, ShadowRaysAreBounded)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset rs = rayset::generate(ray_distribution::shadow_ao, f.m, f.tree, f.cam, small_args());
	ASSERT_GT(rs.size(), 0u);

	const f32 diag = length(f.m.bounds().extent());
	for (u32 i = 0; i < rs.size(); ++i)
	{
		ASSERT_LT(rs.t_max[i], t_max_default) << "AO rays must be bounded, not open-ended";
		ASSERT_LE(rs.t_max[i], diag * 0.1f + 1e-3f);
	}
}

TEST(RaySet, DirectionsAreFinite)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	for (ray_distribution dist : all_ray_distributions)
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, small_args());
		for (u32 i = 0; i < rs.size(); ++i)
		{
			const vec3 d = rs.d[i];
			ASSERT_TRUE(std::isfinite(d.x) && std::isfinite(d.y) && std::isfinite(d.z))
				<< to_string(dist) << " ray " << i;
			ASSERT_GT(length(d), 0.0f) << to_string(dist) << " ray " << i;
		}
	}
}

TEST(RaySet, IncoherentOriginsAreInsideTheSceneBounds)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset rs = rayset::generate(ray_distribution::incoherent, f.m, f.tree, f.cam, small_args());
	ASSERT_EQ(rs.size(), 4096u);

	const aabb& b = f.m.bounds();
	for (u32 i = 0; i < rs.size(); ++i)
	{
		ASSERT_GE(rs.o[i].x, b.min.x); ASSERT_LE(rs.o[i].x, b.max.x);
		ASSERT_GE(rs.o[i].y, b.min.y); ASSERT_LE(rs.o[i].y, b.max.y);
		ASSERT_GE(rs.o[i].z, b.min.z); ASSERT_LE(rs.o[i].z, b.max.z);
	}
}

// determinism and hashing

TEST(RaySet, GenerationIsDeterministic)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	for (ray_distribution dist : all_ray_distributions)
	{
		const rayset a = rayset::generate(dist, f.m, f.tree, f.cam, small_args());
		const rayset b = rayset::generate(dist, f.m, f.tree, f.cam, small_args());

		ASSERT_EQ(a.size(), b.size()) << to_string(dist);
		EXPECT_EQ(a.hash(), b.hash()) << to_string(dist) << " is not reproducible";
	}
}

TEST(RaySet, DifferentSeedsGiveDifferentRays)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	rayset_args a1 = small_args();
	rayset_args a2 = small_args();
	a2.seed = a1.seed ^ 0xabcdefull;

	const rayset r1 = rayset::generate(ray_distribution::diffuse_1, f.m, f.tree, f.cam, a1);
	const rayset r2 = rayset::generate(ray_distribution::diffuse_1, f.m, f.tree, f.cam, a2);

	EXPECT_NE(r1.hash(), r2.hash());
}

TEST(RaySet, HashDistinguishesDistributions)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset a = rayset::generate(ray_distribution::diffuse_1, f.m, f.tree, f.cam, small_args());
	const rayset b = rayset::generate(ray_distribution::reflection, f.m, f.tree, f.cam, small_args());
	EXPECT_NE(a.hash(), b.hash());
}

TEST(RaySet, HashIsSensitiveToASingleRay)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	rayset a = rayset::generate(ray_distribution::primary, f.m, f.tree, f.cam, small_args());
	const u64 before = a.hash();
	a.d[a.size() / 2].x = std::nextafterf(a.d[a.size() / 2].x, 1.0f);
	EXPECT_NE(a.hash(), before);
}

// serialization
TEST(RaySet, RoundTripsThroughDisk)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	for (ray_distribution dist : all_ray_distributions)
	{
		rayset a = rayset::generate(dist, f.m, f.tree, f.cam, small_args());
		a.scene = "teapot.obj";

		const std::string path = temp_path(to_string(dist));
		ASSERT_TRUE(a.save(path)) << to_string(dist);

		rayset b;
		ASSERT_TRUE(b.load(path)) << to_string(dist);

		EXPECT_EQ(a.size(), b.size());
		EXPECT_EQ(a.hash(), b.hash()) << "serialization is not bit-exact";
		EXPECT_EQ(b.distribution, dist);
		EXPECT_EQ(b.scene, "teapot.obj");
		EXPECT_EQ(b.seed, a.seed);

		std::filesystem::remove(path);
	}
}

TEST(RaySet, LoadRejectsMissingAndForeignFiles)
{
	rayset rs;
	EXPECT_FALSE(rs.load("definitely-not-a-rayset.rays"));
	const std::string path = temp_path("garbage");
	{
		std::ofstream f(path, std::ios::binary);
		const char junk[64] = {};
		f.write(junk, sizeof(junk));
	}
	EXPECT_FALSE(rs.load(path));
	std::filesystem::remove(path);
}

TEST(RaySet, FilenameEncodesTheParametersThatChangeTheRays)
{
	rayset_args a = small_args();
	rayset_args b = small_args();
	b.bounces = a.bounces + 1;

	const std::string na = rayset_filename("teapot.obj", ray_distribution::diffuse_n, a);
	const std::string nb = rayset_filename("teapot.obj", ray_distribution::diffuse_n, b);

	EXPECT_NE(na, nb) << "a cached set must not be reused after its parameters change";
	EXPECT_NE(na, rayset_filename("teapot.obj", ray_distribution::diffuse_1, a));
}

TEST(RaySet, DistributionNamesRoundTrip)
{
	for (ray_distribution d : all_ray_distributions)
	{
		ray_distribution parsed;
		ASSERT_TRUE(ray_distribution_from_string(to_string(d), parsed)) << to_string(d);
		EXPECT_EQ(parsed, d);
	}

	ray_distribution ignored;
	EXPECT_FALSE(ray_distribution_from_string("not_a_distribution", ignored));
}

// tracing a ray set
TEST(RaySet, TraceRaysetAgreesWithCameraTrace)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset primary = rayset::generate(ray_distribution::primary, f.m, f.tree, f.cam, small_args());

	const trace_result via_set = trace_rayset(f.tree, f.m, primary, 1);
	const trace_result via_cam = trace_only(f.tree, f.m, f.cam, 1);
	EXPECT_EQ(via_set.rays, via_cam.rays);
	EXPECT_EQ(via_set.hits, via_cam.hits);
	EXPECT_EQ(via_set.node_steps, via_cam.node_steps);
	EXPECT_EQ(via_set.prim_steps, via_cam.prim_steps);
}

TEST(RaySet, IncoherentRaysCostMoreThanPrimary)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset primary = rayset::generate(ray_distribution::primary, f.m, f.tree, f.cam, small_args());
	const rayset incoherent = rayset::generate(ray_distribution::incoherent, f.m, f.tree, f.cam, small_args());

	const trace_result p = trace_rayset(f.tree, f.m, primary, 1);
	const trace_result i = trace_rayset(f.tree, f.m, incoherent, 1);
	EXPECT_GT(i.node_steps_per_ray(), p.node_steps_per_ray());
}

TEST(RaySet, OccludeRaysetRespectsBoundedTMax)
{
	fixture f;
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	const rayset ao = rayset::generate(ray_distribution::shadow_ao, f.m, f.tree, f.cam, small_args());
	ASSERT_GT(ao.size(), 0u);

	const trace_result occ = occlude_rayset(f.tree, f.m, ao, 1);
	EXPECT_EQ(occ.rays, ao.size());
	rayset unbounded = ao;
	for (u32 i = 0; i < unbounded.size(); ++i) unbounded.t_max[i] = t_max_default;

	const trace_result open = occlude_rayset(f.tree, f.m, unbounded, 1);
	EXPECT_LT(occ.node_steps, open.node_steps);
}

TEST(RaySet, RaysetThroughputMatchesCameraThroughput)
{
	fixture f(256);
	if (!f.ok) GTEST_SKIP() << "teapot.obj not found";

	rayset_args a = small_args(256);
	const rayset primary = rayset::generate(ray_distribution::primary, f.m, f.tree, f.cam, a);

	const trace_result via_set = trace_rayset(f.tree, f.m, primary, 0, 3);
	const trace_result via_cam = trace_only(f.tree, f.m, f.cam, 0);

	// Same rays, same work: step counts must be identical.
	ASSERT_EQ(via_set.rays, via_cam.rays);
	ASSERT_EQ(via_set.node_steps, via_cam.node_steps);

	const double ratio = via_cam.mrays_per_second() / via_set.mrays_per_second();
	EXPECT_LT(ratio, 3.0) << "ray-set path is " << ratio << "x slower than the camera path on identical rays;" " suspect per-ray locking";
}
