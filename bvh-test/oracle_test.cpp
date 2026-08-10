#include <core/mode.h>
#include <reference/brute_force.h>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace bvh;

namespace
{
	mesh make_mesh(std::vector<vec3> verts, std::vector<uvec3> tris)
	{
		mesh m;
		m.vertices = std::move(verts);
		m.vertex_indices = std::move(tris);
		const size_t n = m.vertex_indices.size();
		m.normal_indices.assign(n, uvec3(invalid_id, invalid_id, invalid_id));
		m.tex_coord_indices.assign(n, uvec3(invalid_id, invalid_id, invalid_id));
		m.material_indices.assign(n, invalid_id);
		m.compute_bounds();
		return m;
	}

	// Two coincident triangles at the same depth: both are equally the nearest hit.
	mesh duplicate_triangles()
	{
		return make_mesh({ vec3(0.0f, 0.0f, 1.0f), vec3(1.0f, 0.0f, 1.0f), vec3(0.0f, 1.0f, 1.0f) }, { uvec3(0, 1, 2), uvec3(0, 1, 2) });
	}

	// Two triangles meeting along a shared edge, forming a quad.
	mesh shared_edge_quad()
	{
		return make_mesh({ vec3(-1.0f, -1.0f, 1.0f), vec3(1.0f, -1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f), vec3(-1.0f, 1.0f, 1.0f) }, { uvec3(0, 1, 2), uvec3(0, 2, 3) });
	}
} // namespace

// tolerance helper
TEST(OracleTolerance, ExactEqualityAlwaysAgrees)
{
	EXPECT_TRUE(t_agrees(1.0f, 1.0f));
	EXPECT_TRUE(t_agrees(0.0f, 0.0f));
	EXPECT_TRUE(t_agrees(1e6f, 1e6f));
}

TEST(OracleTolerance, IsRelativeNotAbsolute)
{
	// A 0.5 absolute difference is a mismatch at unit scale and acceptable at 1e6 scale.
	EXPECT_FALSE(t_agrees(1.0f, 1.5f));
	EXPECT_TRUE(t_agrees(1.0e6f, 1.0e6f + 0.5f));
}

TEST(OracleTolerance, RejectsGenuineDisagreement)
{
	EXPECT_FALSE(t_agrees(1.0f, 2.0f));
	EXPECT_FALSE(t_agrees(1.0e6f, 2.0e6f));
}

TEST(OracleTolerance, NaNNeverAgrees)
{
	const f32 nan = std::nanf("");
	EXPECT_FALSE(t_agrees(nan, 1.0f));
	EXPECT_FALSE(t_agrees(1.0f, nan));
	EXPECT_FALSE(t_agrees(nan, nan)) << "two NaNs are not evidence of agreement";
}

// tie sets
TEST(OracleTieSet, UniqueNearestHitHasASetOfOne)
{
	const mesh m = shared_edge_quad();
	// Straight through the interior of triangle 0, well away from the diagonal.
	const ray r(vec3(0.4f, -0.4f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit ref; ref.t = r.t_max;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(m, r, ref, s));

	std::vector<u32> tie;
	nearest_tie_set(m, r, ref.t, tie);
	EXPECT_EQ(tie.size(), 1u) << "unique nearest hit: exact id comparison is valid here";
}

TEST(OracleTieSet, DuplicateTrianglesTie)
{
	const mesh m = duplicate_triangles();
	const ray r(vec3(0.2f, 0.2f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit ref; ref.t = r.t_max;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(m, r, ref, s));

	std::vector<u32> tie;
	nearest_tie_set(m, r, ref.t, tie);
	EXPECT_EQ(tie.size(), 2u) << "coincident triangles are both correct nearest hits";
}

// The case the old criterion got wrong: a different-but-correct id must pass.
TEST(OracleCompare, AcceptsEitherOfTwoTiedPrimitives)
{
	const mesh m = duplicate_triangles();
	const ray r(vec3(0.2f, 0.2f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit ref; ref.t = r.t_max;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(m, r, ref, s));

	std::vector<u32> scratch;
	for (u32 id : {0u, 1u})
	{
		const oracle_result res = compare_against_oracle(m, r, true, ref.t, id, scratch);
		EXPECT_TRUE(res.agree) << "id " << id << " is a legitimate nearest hit";
		EXPECT_FALSE(res.id_mismatch);
	}

	// Exactly one of them is the id brute force happened to return, so the other
	// must have been accepted via the tie set rather than by equality.
	const bool a = compare_against_oracle(m, r, true, ref.t, 0u, scratch).tie_resolved;
	const bool b = compare_against_oracle(m, r, true, ref.t, 1u, scratch).tie_resolved;
	EXPECT_NE(a, b) << "one by equality, one by tie set";
}

TEST(OracleCompare, StillRejectsAWrongPrimitive)
{
	// The relaxation must not make the oracle toothless: an id that is not a
	// nearest hit at all is still a failure.
	const mesh m = shared_edge_quad();
	const ray r(vec3(0.4f, -0.4f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit ref; ref.t = r.t_max;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(m, r, ref, s));

	const u32 wrong = ref.id == 0u ? 1u : 0u;
	std::vector<u32> scratch;
	const oracle_result res = compare_against_oracle(m, r, true, ref.t, wrong, scratch);

	EXPECT_FALSE(res.agree);
	EXPECT_TRUE(res.id_mismatch);
}

TEST(OracleCompare, RejectsHitMissDisagreement)
{
	const mesh m = shared_edge_quad();
	const ray miss(vec3(10.0f, 10.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	std::vector<u32> scratch;
	const oracle_result res = compare_against_oracle(m, miss, true, 1.0f, 0u, scratch);

	EXPECT_FALSE(res.agree);
	EXPECT_TRUE(res.miss_mismatch);
}

TEST(OracleCompare, RejectsDistantT)
{
	const mesh m = shared_edge_quad();
	const ray r(vec3(0.4f, -0.4f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit ref; ref.t = r.t_max;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(m, r, ref, s));

	std::vector<u32> scratch;
	const oracle_result res =
		compare_against_oracle(m, r, true, ref.t * 2.0f, ref.id, scratch);

	EXPECT_FALSE(res.agree);
	EXPECT_TRUE(res.t_mismatch);
}

// degenerate ray/geometry cases

TEST(OracleDegenerate, SharedEdgeRayIsHandledConsistently)
{
	const mesh m = shared_edge_quad();
	const ray r(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit ref; ref.t = r.t_max;
	null_stats s;
	const bool hit_any = intersect_brute_force(m, r, ref, s);

	std::vector<u32> scratch;
	const oracle_result res =
		compare_against_oracle(m, r, hit_any, ref.t, hit_any ? ref.id : invalid_id, scratch);
	EXPECT_TRUE(res.agree);
}

TEST(OracleDegenerate, SignedZeroDirectionMatchesPositiveZero)
{
	const aabb box(vec3(-1.0f, -1.0f, -1.0f), vec3(1.0f, 1.0f, 1.0f));

	const ray pos(vec3(-5.0f, 0.5f, 0.0f), vec3(1.0f, 0.0f, 0.0f));
	const ray neg(vec3(-5.0f, 0.5f, 0.0f), vec3(1.0f, -0.0f, 0.0f));

	const f32 a = intersect<robust_mode>(box, pos, rcp(pos.d));
	const f32 b = intersect<robust_mode>(box, neg, rcp(neg.d));

	EXPECT_FLOAT_EQ(a, b);
	EXPECT_FLOAT_EQ(a, 4.0f);
}

TEST(OracleDegenerate, LargeCoordinatesDoNotLoseTheHit)
{
	constexpr f32 k = 1.0e4f;
	const mesh m = make_mesh({ vec3(k, k, k + 1.0f), vec3(k + 1.0f, k, k + 1.0f), vec3(k, k + 1.0f, k + 1.0f) }, { uvec3(0, 1, 2) });

	const ray r(vec3(k + 0.25f, k + 0.25f, k - 1.0f), vec3(0.0f, 0.0f, 1.0f));

	hit h; h.t = r.t_max;
	null_stats s;
	ASSERT_TRUE(intersect_brute_force(m, r, h, s));
	EXPECT_NEAR(h.t, 2.0f, 2.0f * 1e-3f);
}

TEST(OracleDegenerate, InfiniteAndNaNRayComponentsNeverProduceAHit)
{
	const mesh m = shared_edge_quad();
	const f32 inf = INFINITY;
	const f32 nan = std::nanf("");

	for (const vec3& d : { vec3(inf, 0.0f, 1.0f), vec3(nan, 0.0f, 1.0f), vec3(0.0f, 0.0f, nan) })
	{
		const ray r(vec3(0.0f, 0.0f, -1.0f), d);
		hit h; h.t = r.t_max;
		null_stats s;
		intersect_brute_force<robust_mode>(m, r, h, s);
		EXPECT_FALSE(std::isnan(h.t)) << "NaN reached the hit record";
	}
}
