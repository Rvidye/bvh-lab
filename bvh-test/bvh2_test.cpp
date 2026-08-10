#include <build/bvh2_builder.h>
#include <core/traverse_bvh2.h>
#include <eval/quality.h>
#include <eval/trace.h>
#include <reference/brute_force.h>
#include <util/camera.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>
#include <vector>

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

	bool load(mesh& m, const char* name = "teapot.obj")
	{
		const std::string p = find_scene(name);
		if (p.empty()) return false;
		return m.load_obj(p);
	}

	const split_method all_methods[] = {
		split_method::median,
		split_method::binned_sah_arches,
		split_method::binned_sah,
		split_method::sweep_sah,
	};

	// Two quads at different depths, so a hand-checkable tree exists without a file.
	mesh two_quads()
	{
		mesh m;
		m.vertices = {
			vec3(-1.0f, -1.0f, 1.0f), vec3(1.0f, -1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f), vec3(-1.0f, 1.0f, 1.0f),
			vec3(-1.0f, -1.0f, 5.0f), vec3(1.0f, -1.0f, 5.0f), vec3(1.0f, 1.0f, 5.0f), vec3(-1.0f, 1.0f, 5.0f),
		};
		m.vertex_indices = { uvec3(0, 1, 2), uvec3(0, 2, 3), uvec3(4, 5, 6), uvec3(4, 6, 7) };
		m.normal_indices.assign(4, uvec3(invalid_id, invalid_id, invalid_id));
		m.tex_coord_indices.assign(4, uvec3(invalid_id, invalid_id, invalid_id));
		m.material_indices.assign(4, invalid_id);
		m.compute_bounds();
		return m;
	}

} // namespace

  // layout

TEST(BVH2Layout, MatchesArches)
{
	EXPECT_EQ(sizeof(bvh_ptr), 4u);
	EXPECT_EQ(sizeof(bvh2_node), 32u);
	EXPECT_EQ(alignof(bvh2_node), 32u);
}

TEST(BVH2Layout, PointerFieldsAliasRaw)
{
	bvh_ptr p;
	p.is_int = 1;
	p.child_cnt = 2;
	p.child_idx = 12345;

	EXPECT_EQ(p.is_int, 1u);
	EXPECT_EQ(p.child_cnt, 2u);
	EXPECT_EQ(p.child_idx, 12345u);

	bvh_ptr q;
	q.raw = p.raw;
	EXPECT_EQ(q.child_idx, 12345u);
	EXPECT_EQ(q.prim_cnt, 2u) << "prim_cnt and child_cnt occupy the same field";
}

TEST(BVH2Layout, IndexFieldHoldsTwentySixBits)
{
	bvh_ptr p;
	p.child_idx = (1u << 26) - 1u;
	EXPECT_EQ(p.child_idx, (1u << 26) - 1u);
}

// construction invariants
TEST(BVH2Build, HandBuiltSceneProducesSaneTree)
{
	mesh m = two_quads();

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);

	EXPECT_FALSE(tree.empty());
	EXPECT_EQ(tree.report().leaf_count, 4u) << "one leaf per triangle at max_leaf_size 1";
	EXPECT_EQ(tree.report().node_count, 7u) << "4 leaves + 3 interior for a binary tree";
	EXPECT_EQ(tree.report().interior_count, 3u);
}

TEST(BVH2Build, ChildrenAlwaysFollowParents)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	for (split_method method : all_methods)
	{
		mesh copy = m;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(copy, a);

		const std::vector<bvh2_node>& nodes = tree.nodes();
		for (u32 i = 0; i < nodes.size(); ++i)
		{
			if (!nodes[i].ptr.is_int) continue;
			for (u32 c = 0; c < nodes[i].ptr.child_cnt; ++c)
				ASSERT_GT(nodes[i].ptr.child_idx + c, i)
				<< to_string(method) << ": node " << i << " child " << c;
		}
	}
}

TEST(BVH2Build, EveryPrimitiveAppearsInExactlyOneLeaf)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	for (split_method method : all_methods)
	{
		mesh copy = m;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(copy, a);

		std::vector<u32> seen(copy.triangle_count(), 0u);
		for (const bvh2_node& node : tree.nodes())
		{
			if (node.ptr.is_int) continue;
			for (u32 p = 0; p < node.ptr.prim_cnt; ++p)
			{
				ASSERT_LT(node.ptr.prim_idx + p, copy.triangle_count());
				++seen[node.ptr.prim_idx + p];
			}
		}

		for (u32 i = 0; i < seen.size(); ++i)
			ASSERT_EQ(seen[i], 1u) << to_string(method) << ": slot " << i << " covered " << seen[i] << " times";
	}
}

TEST(BVH2Build, PermutationIsABijection)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);

	std::vector<u32> seen(m.triangle_count(), 0u);
	for (u32 id : tree.prim_indices())
	{
		ASSERT_LT(id, m.triangle_count());
		++seen[id];
	}
	for (u32 c : seen) ASSERT_EQ(c, 1u);
}

TEST(BVH2Build, ParentBoundsContainChildBounds)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);

	const std::vector<bvh2_node>& nodes = tree.nodes();
	for (const bvh2_node& node : nodes)
	{
		if (!node.ptr.is_int) continue;
		for (u32 c = 0; c < node.ptr.child_cnt; ++c)
		{
			const aabb& kid = nodes[node.ptr.child_idx + c].bounds;
			ASSERT_LE(node.bounds.min.x, kid.min.x);
			ASSERT_LE(node.bounds.min.y, kid.min.y);
			ASSERT_LE(node.bounds.min.z, kid.min.z);
			ASSERT_GE(node.bounds.max.x, kid.max.x);
			ASSERT_GE(node.bounds.max.y, kid.max.y);
			ASSERT_GE(node.bounds.max.z, kid.max.z);
		}
	}
}

TEST(BVH2Build, RootBoundsMatchMeshBounds)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);

	const aabb& root = tree.nodes()[0].bounds;
	const aabb& mb = m.bounds();

	EXPECT_NEAR(root.min.x, mb.min.x, 1e-4f);
	EXPECT_NEAR(root.max.y, mb.max.y, 1e-4f);
	EXPECT_NEAR(root.max.z, mb.max.z, 1e-4f);
}

TEST(BVH2Build, DepthFitsTheTraversalStack)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	for (split_method method : all_methods)
	{
		mesh copy = m;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(copy, a);

		ASSERT_LT(tree.report().max_depth + 2u, bvh2_stack_size) << to_string(method);
	}
}

TEST(BVH2Build, ApplyReorderMakesPrimIndexIdentity)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	for (u32 i = 0; i < m.triangle_count(); ++i) ASSERT_EQ(tree.prim_index(i), i);
}

// correctness against the oracle
TEST(BVH2Traverse, EveryBuilderMatchesBruteForce)
{
	mesh base;
	if (!load(base)) GTEST_SKIP() << "teapot.obj not found";

	const u32 res = 64;

	for (split_method method : all_methods)
	{
		mesh m = base;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(m, a);
		tree.apply_reorder(m);

		const camera     cam = camera::frame_bounds(m.bounds(), res, res);
		const bvh2_view  view = tree.view();
		const auto prims = make_prims(m, tree);

		u32 mismatches = 0;
		std::vector<u32> scratch;
		for (u32 j = 0; j < res; ++j)
		{
			for (u32 i = 0; i < res; ++i)
			{
				const ray r = cam.generate_ray_through_pixel(i, j);

				hit fast; null_stats s1;
				intersect(view, r, fast, prims, s1);

				const u32 fast_id = fast.valid() ? tree.prim_index(fast.id) : invalid_id;
				// Tie-aware: exact id only where the nearest hit is unique.
				if (!compare_against_oracle(m, r, fast.valid(), fast.t, fast_id, scratch).agree)
					++mismatches;
			}
		}

		EXPECT_EQ(mismatches, 0u) << to_string(method) << " disagrees with the oracle";
	}
}

TEST(BVH2Traverse, OccludedAgreesWithClosestHit)
{
	mesh m = two_quads();
	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const bvh2_view  view = tree.view();
	const auto prims = make_prims(m, tree);
	null_stats s;

	const ray blocked(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_TRUE(occluded(view, blocked, prims, s));

	const ray clear(vec3(10.0f, 10.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	EXPECT_FALSE(occluded(view, clear, prims, s));
}

TEST(BVH2Traverse, StepSemanticsMatchArches)
{
	mesh m = two_quads();
	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const bvh2_view  view = tree.view();
	const auto prims = make_prims(m, tree);

	hit h;
	trace_stats stats;
	const ray r(vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
	ASSERT_TRUE(intersect(view, r, h, prims, stats));

	EXPECT_GT(stats.node_steps, 0u);
	EXPECT_GT(stats.prim_steps, 0u);
	EXPECT_EQ(stats.total_steps(), stats.node_steps + stats.prim_steps);
	EXPECT_EQ(stats.box_tests, stats.node_steps * 2u - 1u);
}

TEST(BVH2Traverse, OrderedTraversalPrunesMostOfTheTree)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const camera cam = camera::frame_bounds(m.bounds(), 64, 64);
	const trace_result tr = trace_only(tree, m, cam, 1);

	EXPECT_LT(tr.node_steps_per_ray(), double(tree.report().interior_count) / 10.0) << "ordered traversal should visit a small fraction of interior nodes";
}

TEST(BVH2Traverse, EmptyTreeIsSafe)
{
	bvh2_view empty;
	const ray r(vec3(0.0f), vec3(0.0f, 0.0f, 1.0f));
	hit h;
	null_stats s;
	auto never = [](u32, const ray&, hit&) { return false; };

	EXPECT_FALSE(intersect(empty, r, h, never, s));
	EXPECT_FALSE(occluded(empty, r, never, s));
}

// refit

TEST(BVH2Refit, IsIdempotentOnUnchangedGeometry)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const std::vector<bvh2_node> before = tree.nodes();
	tree.refit(m);
	const std::vector<bvh2_node>& after = tree.nodes();

	ASSERT_EQ(before.size(), after.size());
	for (size_t i = 0; i < before.size(); ++i)
	{
		ASSERT_EQ(before[i].ptr.raw, after[i].ptr.raw) << "refit must not touch topology";
		ASSERT_FLOAT_EQ(before[i].bounds.min.x, after[i].bounds.min.x) << "node " << i;
		ASSERT_FLOAT_EQ(before[i].bounds.max.z, after[i].bounds.max.z) << "node " << i;
	}
}

TEST(BVH2Refit, TracksDeformedGeometry)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const f32 before_max_y = tree.nodes()[0].bounds.max.y;
	for (vec3& v : m.vertices) v.y *= 2.0f;
	m.compute_bounds();
	tree.refit(m);

	const f32 after_max_y = tree.nodes()[0].bounds.max.y;
	EXPECT_GT(after_max_y, before_max_y);
	EXPECT_NEAR(after_max_y, m.bounds().max.y, 1e-3f);
	const camera     cam = camera::frame_bounds(m.bounds(), 32, 32);
	const bvh2_view  view = tree.view();
	const auto prims = make_prims(m, tree);

	u32 mismatches = 0;
	std::vector<u32> scratch;
	for (u32 j = 0; j < 32; ++j)
		for (u32 i = 0; i < 32; ++i)
		{
			const ray r = cam.generate_ray_through_pixel(i, j);
			hit fast; null_stats s1; intersect(view, r, fast, prims, s1);
			const u32 fast_id = fast.valid() ? tree.prim_index(fast.id) : invalid_id;
			if (!compare_against_oracle(m, r, fast.valid(), fast.t, fast_id, scratch).agree)
				++mismatches;
		}

	EXPECT_EQ(mismatches, 0u) << "refitted tree disagrees with the oracle";
}

// quality metrics

TEST(BVH2Quality, SahCostRanksBuildersCorrectly)
{
	mesh base;
	if (!load(base)) GTEST_SKIP() << "teapot.obj not found";

	auto cost_of = [&](split_method method) {
		mesh m = base;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(m, a);
		tree.apply_reorder(m);
		return evaluate(tree, m).sah_cost;
		};

	const double median = cost_of(split_method::median);
	const double binned = cost_of(split_method::binned_sah);
	const double sweep = cost_of(split_method::sweep_sah);

	EXPECT_GT(median, binned) << "median split must be measurably worse than binned SAH";
	EXPECT_GT(median, sweep) << "median split must be measurably worse than sweep SAH";
	EXPECT_LT(std::abs(binned - sweep) / sweep, 0.15) << "binned and sweep SAH should be within 15% of each other";
}

TEST(BVH2Quality, EpoRanksBuildersCorrectly)
{
	mesh base;
	if (!load(base)) GTEST_SKIP() << "teapot.obj not found";

	auto epo_of = [&](split_method method) {
		mesh m = base;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(m, a);
		tree.apply_reorder(m);
		quality_args qa;
		qa.compute_epo = true;
		return evaluate(tree, m, qa).epo;
		};

	const double median = epo_of(split_method::median);
	const double binned = epo_of(split_method::binned_sah);

	EXPECT_GT(median, 0.0) << "EPO must be non-zero: real trees do have overlap";
	EXPECT_GT(median, binned) << "a worse tree must show more end-point overlap";
}

TEST(BVH2Quality, FewerStepsForBetterTrees)
{
	mesh base;
	if (!load(base)) GTEST_SKIP() << "teapot.obj not found";

	auto steps_of = [&](split_method method) {
		mesh m = base;
		bvh2 tree;
		build_args a;
		a.method = method;
		a.silent = true;
		tree.build(m, a);
		tree.apply_reorder(m);
		const camera cam = camera::frame_bounds(m.bounds(), 64, 64);
		return trace_only(tree, m, cam, 1).node_steps_per_ray();
		};

	EXPECT_GT(steps_of(split_method::median), steps_of(split_method::binned_sah));
}

TEST(BVH2Quality, ReportsSizeAndShape)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const quality_metrics q = evaluate(tree, m);

	EXPECT_EQ(q.node_count, tree.report().node_count);
	EXPECT_EQ(q.leaf_count + q.interior_count, q.node_count);
	EXPECT_EQ(q.bytes, tree.nodes().size() * sizeof(bvh2_node));
	EXPECT_GT(q.sah_cost, 0.0);
	EXPECT_GT(q.sah_cost_arches, 0.0);
	EXPECT_NEAR(q.mean_leaf_size, 1.0, 1e-6) << "max_leaf_size 1 means one prim per leaf";
}

TEST(BVH2Quality, BvhBeatsBruteForceByALot)
{
	mesh m;
	if (!load(m)) GTEST_SKIP() << "teapot.obj not found";

	bvh2 tree;
	build_args a;
	a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const camera cam = camera::frame_bounds(m.bounds(), 64, 64);
	const trace_result tr = trace_only(tree, m, cam, 1);
	EXPECT_LT(tr.tri_tests_per_ray(), double(m.triangle_count()) / 10.0);
}

TEST(BVH2Build, IsDeterministic)
{
	mesh base;
	if (!load(base)) GTEST_SKIP() << "teapot.obj not found";

	for (split_method method : all_methods)
	{
		mesh a = base, b = base;
		bvh2 ta, tb;
		build_args args;
		args.method = method;
		args.silent = true;

		ta.build(a, args);
		tb.build(b, args);

		ASSERT_EQ(ta.nodes().size(), tb.nodes().size()) << to_string(method);
		for (size_t i = 0; i < ta.nodes().size(); ++i)
		{
			ASSERT_EQ(ta.nodes()[i].ptr.raw, tb.nodes()[i].ptr.raw)
				<< to_string(method) << " node " << i;
			ASSERT_FLOAT_EQ(ta.nodes()[i].bounds.min.x, tb.nodes()[i].bounds.min.x);
			ASSERT_FLOAT_EQ(ta.nodes()[i].bounds.max.z, tb.nodes()[i].bounds.max.z);
		}
		ASSERT_EQ(ta.prim_indices(), tb.prim_indices()) << to_string(method);
	}
}
