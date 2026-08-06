#include <util/mesh.h>

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

} // namespace

TEST(Mesh, LoadsTeapot)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	EXPECT_EQ(m.triangle_count(), 1024u);
	EXPECT_FALSE(m.empty());
	EXPECT_FALSE(m.normals.empty()) << "teapot.obj carries vertex normals";

	// Every per-triangle array must be the same length or indexing them
	// together in the builder walks off the end.
	EXPECT_EQ(m.normal_indices.size(), m.vertex_indices.size());
	EXPECT_EQ(m.tex_coord_indices.size(), m.vertex_indices.size());
	EXPECT_EQ(m.material_indices.size(), m.vertex_indices.size());
}

TEST(Mesh, LoadsMeshWithoutNormals)
{
	const std::string path = find_scene("bunny.obj");
	if (path.empty()) GTEST_SKIP() << "bunny.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	EXPECT_EQ(m.triangle_count(), 4968u);
	EXPECT_TRUE(m.normals.empty()) << "bunny.obj carries no vertex normals";
}

TEST(Mesh, MissingLoadFails)
{
	mesh m;
	EXPECT_FALSE(m.load_obj("definitely-not-a-real-file.obj"));
}

TEST(Mesh, AbsentAttributesAreInvalidId)
{
	const std::string path = find_scene("bunny.obj");
	if (path.empty()) GTEST_SKIP() << "bunny.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	bool saw_invalid = false;
	for (const uvec3& ni : m.normal_indices)
	{
		if (ni.x == invalid_id) { saw_invalid = true; break; }
	}
	EXPECT_TRUE(saw_invalid) << "absent normal indices did not survive as invalid_id";
}

TEST(Mesh, VertexIndicesAreInRange)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const u32 nverts = static_cast<u32>(m.vertices.size());
	for (const uvec3& vi : m.vertex_indices)
	{
		ASSERT_LT(vi.x, nverts);
		ASSERT_LT(vi.y, nverts);
		ASSERT_LT(vi.z, nverts);
	}
}

TEST(Mesh, BoundsContainEveryVertex)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const aabb& b = m.bounds();
	ASSERT_FALSE(b.empty());

	for (const vec3& v : m.vertices)
	{
		ASSERT_GE(v.x, b.min.x); ASSERT_LE(v.x, b.max.x);
		ASSERT_GE(v.y, b.min.y); ASSERT_LE(v.y, b.max.y);
		ASSERT_GE(v.z, b.min.z); ASSERT_LE(v.z, b.max.z);
	}
}

TEST(Mesh, ScaleScalesBounds)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh a, b;
	ASSERT_TRUE(a.load_obj(path, 1.0f));
	ASSERT_TRUE(b.load_obj(path, 2.0f));

	EXPECT_NEAR(length(b.bounds().extent()), 2.0f * length(a.bounds().extent()), 1e-3f);
}

TEST(Mesh, ShadingNormalFallsBackToGeometricWithoutNormals)
{
	const std::string path = find_scene("bunny.obj");
	if (path.empty()) GTEST_SKIP() << "bunny.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const vec3 n = m.shading_normal(0, vec2(0.25f, 0.25f));
	EXPECT_NEAR(length(n), 1.0f, 1e-5f);

	const vec3 g = m.geometric_normal(0);
	EXPECT_NEAR(std::abs(dot(n, g)), 1.0f, 1e-5f);
}

TEST(Mesh, ShadingNormalIsUnitLength)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	for (u32 i = 0; i < m.triangle_count(); i += 37)
	{
		const vec3 n = m.shading_normal(i, vec2(0.3f, 0.4f));
		ASSERT_NEAR(length(n), 1.0f, 1e-4f) << "triangle " << i;
	}
}

TEST(Mesh, ShadingNormalBarycentricWeightingMatchesArches)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	// Find a triangle whose three vertex normals actually differ, otherwise the
	// weighting is unobservable.
	for (u32 i = 0; i < m.triangle_count(); ++i)
	{
		const uvec3& ni = m.normal_indices[i];
		if (ni.x == invalid_id) continue;

		const vec3 n0 = m.normals[ni.x];
		const vec3 n1 = m.normals[ni.y];
		if (length(n0 - n1) < 0.2f) continue;

		// bc = (1, 0) must reproduce vertex 0's normal.
		const vec3 at_v0 = m.shading_normal(i, vec2(1.0f, 0.0f));
		EXPECT_NEAR(dot(at_v0, normalize(n0)), 1.0f, 1e-3f);

		// bc = (0, 1) must reproduce vertex 1's normal.
		const vec3 at_v1 = m.shading_normal(i, vec2(0.0f, 1.0f));
		EXPECT_NEAR(dot(at_v1, normalize(n1)), 1.0f, 1e-3f);
		return;
	}
	GTEST_SKIP() << "no triangle with sufficiently distinct vertex normals";
}

TEST(Mesh, ReorderAppliesPermutation)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const std::vector<uvec3> original = m.vertex_indices;
	const u32 n = m.triangle_count();

	// Reverse the order.
	std::vector<u32> perm(n);
	for (u32 i = 0; i < n; ++i) perm[i] = n - 1u - i;

	m.reorder(perm);

	for (u32 i = 0; i < n; ++i)
	{
		ASSERT_EQ(m.vertex_indices[i].x, original[n - 1u - i].x) << "triangle " << i;
		ASSERT_EQ(m.vertex_indices[i].y, original[n - 1u - i].y);
		ASSERT_EQ(m.vertex_indices[i].z, original[n - 1u - i].z);
	}
}

TEST(Mesh, ReorderIsInvolutiveForAReversal)
{
	const std::string path = find_scene("teapot.obj");
	if (path.empty()) GTEST_SKIP() << "teapot.obj not found";

	mesh m;
	ASSERT_TRUE(m.load_obj(path));

	const std::vector<uvec3> original = m.vertex_indices;
	const u32 n = m.triangle_count();

	std::vector<u32> perm(n);
	for (u32 i = 0; i < n; ++i) perm[i] = n - 1u - i;

	m.reorder(perm);
	m.reorder(perm);

	for (u32 i = 0; i < n; ++i) ASSERT_EQ(m.vertex_indices[i].x, original[i].x);
}
