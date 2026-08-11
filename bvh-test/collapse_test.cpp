#include <build/bvh2_builder.h>
#include <build/collapse.h>
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

	// Builds a binary tree and collapses it to 'width'.
	bool make_wide(mesh& m, bvh2& tree, u32 width, collapse_method method, const char* scene = "teapot.obj")
	{
		const std::string p = find_scene(scene);
		if (p.empty() || !m.load_obj(p)) return false;

		build_args ba;
		ba.silent = true;
		tree.build(m, ba);
		tree.apply_reorder(m);

		collapse_args ca;
		ca.width = width;
		ca.method = method;
		ca.silent = true;
		collapse(tree, m, ca);
		return true;
	}

	const collapse_method both_methods[] = { collapse_method::greedy, collapse_method::dynamic_programming };

} // namespace

// structure

TEST(Collapse, RespectsTheWidthCap)
{
	for (u32 width : {4u, 8u})
	{
		for (collapse_method method : both_methods)
		{
			mesh m; bvh2 tree;
			if (!make_wide(m, tree, width, method)) GTEST_SKIP() << "teapot.obj not found";

			for (const bvh2_node& node : tree.nodes())
			{
				if (!node.ptr.is_int) continue;
				ASSERT_GE(u32(node.ptr.child_cnt), 2u) << to_string(method) << " w" << width;
				ASSERT_LE(u32(node.ptr.child_cnt), width) << to_string(method) << " w" << width;
			}
			EXPECT_EQ(tree.width(), width);
		}
	}
}

// collapse() rebuilds the node array from scratch, so the invariant refit depends on has to be re-established, not inherited
TEST(Collapse, PreservesParentBeforeChildren)
{
	for (u32 width : {4u, 8u})
	{
		for (collapse_method method : both_methods)
		{
			mesh m; bvh2 tree;
			if (!make_wide(m, tree, width, method)) GTEST_SKIP() << "teapot.obj not found";

			const std::vector<bvh2_node>& nodes = tree.nodes();
			for (u32 i = 0; i < nodes.size(); ++i)
			{
				if (!nodes[i].ptr.is_int) continue;
				for (u32 c = 0; c < nodes[i].ptr.child_cnt; ++c)
					ASSERT_GT(nodes[i].ptr.child_idx + c, i)
					<< to_string(method) << " w" << width << " node " << i;
			}
		}
	}
}

TEST(Collapse, EveryPrimitiveStillAppearsExactlyOnce)
{
	for (u32 width : {4u, 8u})
	{
		for (collapse_method method : both_methods)
		{
			mesh m; bvh2 tree;
			if (!make_wide(m, tree, width, method)) GTEST_SKIP() << "teapot.obj not found";

			std::vector<u32> seen(m.triangle_count(), 0u);
			for (const bvh2_node& node : tree.nodes())
			{
				if (node.ptr.is_int) continue;
				for (u32 p = 0; p < node.ptr.prim_cnt; ++p)
				{
					ASSERT_LT(node.ptr.prim_idx + p, m.triangle_count());
					++seen[node.ptr.prim_idx + p];
				}
			}
			for (u32 i = 0; i < seen.size(); ++i)
				ASSERT_EQ(seen[i], 1u) << to_string(method) << " w" << width << " slot " << i;
		}
	}
}

TEST(Collapse, ParentBoundsStillContainChildren)
{
	mesh m; bvh2 tree;
	if (!make_wide(m, tree, 8, collapse_method::dynamic_programming))
		GTEST_SKIP() << "teapot.obj not found";

	const std::vector<bvh2_node>& nodes = tree.nodes();
	for (const bvh2_node& node : nodes)
	{
		if (!node.ptr.is_int) continue;
		for (u32 c = 0; c < node.ptr.child_cnt; ++c)
		{
			const aabb& kid = nodes[node.ptr.child_idx + c].bounds;
			ASSERT_LE(node.bounds.min.x, kid.min.x);
			ASSERT_GE(node.bounds.max.z, kid.max.z);
		}
	}
}

TEST(Collapse, WiderTreesAreShallowerAndSmaller)
{
	mesh m2, m4, m8;
	bvh2 t2, t4, t8;
	if (!make_wide(m4, t4, 4, collapse_method::dynamic_programming))
		GTEST_SKIP() << "teapot.obj not found";
	ASSERT_TRUE(make_wide(m8, t8, 8, collapse_method::dynamic_programming));

	{
		const std::string p = find_scene("teapot.obj");
		ASSERT_TRUE(m2.load_obj(p));
		build_args ba; ba.silent = true;
		t2.build(m2, ba);
		t2.apply_reorder(m2);
	}

	// Flattening removes interior levels, so both node count and depth fall
	EXPECT_LT(t4.nodes().size(), t2.nodes().size());
	EXPECT_LT(t8.nodes().size(), t4.nodes().size());
	EXPECT_LT(t8.report().max_depth, t2.report().max_depth);
}

// correctness : the wide tree must still answer the same queries
TEST(Collapse, WideTreesAgreeWithTheOracle)
{
	const u32 res = 64;

	for (u32 width : {4u, 8u})
	{
		for (collapse_method method : both_methods)
		{
			mesh m; bvh2 tree;
			if (!make_wide(m, tree, width, method)) GTEST_SKIP() << "teapot.obj not found";

			const camera    cam = camera::frame_bounds(m.bounds(), res, res);
			const bvh2_view view = tree.view();
			const auto      prims = make_prims(m, tree);

			u32 mismatches = 0;
			std::vector<u32> scratch;
			for (u32 j = 0; j < res; ++j)
				for (u32 i = 0; i < res; ++i)
				{
					const ray r = cam.generate_ray_through_pixel(i, j);
					hit h; null_stats s;
					intersect(view, r, h, prims, s);
					const u32 id = h.valid() ? tree.prim_index(h.id) : invalid_id;
					if (!compare_against_oracle(m, r, h.valid(), h.t, id, scratch).agree)
						++mismatches;
				}

			EXPECT_EQ(mismatches, 0u) << to_string(method) << " w" << width;
		}
	}
}

TEST(Collapse, RefitStillWorksAfterCollapse)
{
	mesh m; bvh2 tree;
	if (!make_wide(m, tree, 8, collapse_method::dynamic_programming))
		GTEST_SKIP() << "teapot.obj not found";

	const f32 before = tree.nodes()[0].bounds.max.y;

	for (vec3& v : m.vertices) v.y *= 2.0f;
	m.compute_bounds();
	tree.refit(m);

	EXPECT_GT(tree.nodes()[0].bounds.max.y, before);
	EXPECT_NEAR(tree.nodes()[0].bounds.max.y, m.bounds().max.y, 1e-3f);
}

TEST(Collapse, IsDeterministic)
{
	for (collapse_method method : both_methods)
	{
		mesh a, b; bvh2 ta, tb;
		if (!make_wide(a, ta, 8, method)) GTEST_SKIP() << "teapot.obj not found";
		ASSERT_TRUE(make_wide(b, tb, 8, method));

		ASSERT_EQ(ta.nodes().size(), tb.nodes().size()) << to_string(method);
		for (size_t i = 0; i < ta.nodes().size(); ++i)
			ASSERT_EQ(ta.nodes()[i].ptr.raw, tb.nodes()[i].ptr.raw)
			<< to_string(method) << " node " << i;
	}
}

// quality
TEST(Collapse, DpIsNotWorseThanGreedy)
{
	for (u32 width : {4u, 8u})
	{
		mesh mg, md; bvh2 tg, td;
		if (!make_wide(mg, tg, width, collapse_method::greedy))
			GTEST_SKIP() << "teapot.obj not found";
		ASSERT_TRUE(make_wide(md, td, width, collapse_method::dynamic_programming));

		const double greedy = evaluate(tg, mg).sah_cost;
		const double dp = evaluate(td, md).sah_cost;

		EXPECT_LE(dp, greedy * 1.001) << "width " << width
			<< ": DP " << dp << " vs greedy " << greedy;
	}
}

TEST(Collapse, FullnessHistogramSumsToInteriorCount)
{
	mesh m; bvh2 tree;
	const std::string p = find_scene("teapot.obj");
	if (p.empty() || !m.load_obj(p)) GTEST_SKIP() << "teapot.obj not found";

	build_args ba; ba.silent = true;
	tree.build(m, ba);
	tree.apply_reorder(m);

	collapse_args ca;
	ca.width = 8;
	ca.silent = true;
	const collapse_report r = collapse(tree, m, ca);

	u32 sum = 0;
	for (u32 i = 0; i <= ca.width; ++i) sum += r.fullness_histogram[i];
	EXPECT_EQ(sum, r.interior_count);

	EXPECT_GE(r.mean_fullness, 2.0);
	EXPECT_LE(r.mean_fullness, double(ca.width));
}

TEST(Collapse, WideTreeVisitsFewerInteriorNodes)
{
	mesh m2, m8; bvh2 t2, t8;
	if (!make_wide(m8, t8, 8, collapse_method::dynamic_programming))
		GTEST_SKIP() << "teapot.obj not found";
	{
		const std::string p = find_scene("teapot.obj");
		ASSERT_TRUE(m2.load_obj(p));
		build_args ba; ba.silent = true;
		t2.build(m2, ba);
		t2.apply_reorder(m2);
	}

	const camera cam2 = camera::frame_bounds(m2.bounds(), 64, 64);
	const camera cam8 = camera::frame_bounds(m8.bounds(), 64, 64);

	const trace_result r2 = trace_only(t2, m2, cam2, 1);
	const trace_result r8 = trace_only(t8, m8, cam8, 1);
	EXPECT_LT(r8.node_steps_per_ray(), r2.node_steps_per_ray());
	EXPECT_GT(r8.box_tests, 0u);
}

TEST(Collapse, OverlapProfileIsPopulated)
{
	mesh m; bvh2 tree;
	if (!make_wide(m, tree, 8, collapse_method::dynamic_programming))
		GTEST_SKIP() << "teapot.obj not found";

	const overlap_profile p = compute_overlap_profile(tree);

	ASSERT_GT(p.depth_count, 0u);
	EXPECT_GT(p.nodes[0], 0u);
	for (u32 d = 0; d < p.depth_count; ++d)
		if (p.nodes[d]) ASSERT_GT(p.mean_overlap[d], 0.0) << "depth " << d;
}
