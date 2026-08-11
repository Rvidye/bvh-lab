#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <util/mesh.h>

#include <gtest/gtest.h>

#include <vector>

using namespace bvh;

// Regression tests for two defects in the overlap profile.
//
// 1. `overlap_profile::nodes` was declared WITHOUT a `{}` initializer while its
//    sibling array had one, so every count -- and every mean derived from it --
//    started from indeterminate stack values. It could report plausible numbers
//    on one run and nonsense on the next, which is worse than crashing.
//
// 2. The quantity named `mean_overlap` was
//        sum(SA(child)) / SA(parent)
//    which is surface-area EXPANSION, not overlap. Perfectly disjoint children
//    produce a large value, so the column could not answer the question it was
//    named for.
//
// "The value is nonzero" is NOT sufficient validation for either: uninitialized
// memory is usually nonzero, and expansion is nonzero for disjoint boxes. These
// tests assert exact expected values on hand-built trees instead.

namespace {

// A tree built by hand so every expected number can be derived on paper.
//
// Root spans [0,4]^3. Two children, each a 2x4x4 half, meeting exactly at x=2:
// disjoint, sharing one face. Each child holds one leaf triangle.
struct hand_tree
{
	mesh m;
	bvh2 tree;

	// `overlap_shift` slides the second child left, so the two children overlap
	// by that much along x. 0 = touching, 2 = fully coincident.
	void build(f32 overlap_shift)
	{
		// Geometry is irrelevant to the profile -- it reads node bounds only --
		// but the mesh must be non-empty for the builder contract.
		m.vertices = {vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f),
		              vec3(3.0f, 0.0f, 0.0f), vec3(4.0f, 0.0f, 0.0f), vec3(3.0f, 1.0f, 0.0f)};
		m.vertex_indices = {uvec3(0, 1, 2), uvec3(3, 4, 5)};
		m.normal_indices.assign(2, uvec3(invalid_id, invalid_id, invalid_id));
		m.tex_coord_indices.assign(2, uvec3(invalid_id, invalid_id, invalid_id));
		m.material_indices.assign(2, invalid_id);
		m.compute_bounds();

		build_args a;
		a.silent = true;
		tree.build(m, a);
		tree.apply_reorder(m);

		// Overwrite the bounds with the exact boxes we want to reason about.
		std::vector<bvh2_node> nodes = tree.nodes();
		ASSERT_GE(nodes.size(), 3u);
		ASSERT_TRUE(nodes[0].ptr.is_int);
		ASSERT_EQ(u32(nodes[0].ptr.child_cnt), 2u);

		nodes[0].bounds = aabb(vec3(0.0f, 0.0f, 0.0f), vec3(4.0f, 4.0f, 4.0f));

		const u32 l = nodes[0].ptr.child_idx + 0;
		const u32 r = nodes[0].ptr.child_idx + 1;
		nodes[l].bounds = aabb(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 4.0f, 4.0f));
		nodes[r].bounds = aabb(vec3(2.0f - overlap_shift, 0.0f, 0.0f), vec3(4.0f, 4.0f, 4.0f));

		tree.replace_nodes(std::move(nodes), 2);
	}
};

} // namespace

// ---------------------------------------------------------------------------
// box_intersection_area -- the primitive the pairwise metric is built on
// ---------------------------------------------------------------------------

TEST(BoxIntersection, DisjointBoxesHaveZeroArea)
{
	const aabb a(vec3(0.0f), vec3(1.0f));
	const aabb b(vec3(2.0f), vec3(3.0f));
	EXPECT_FLOAT_EQ(box_intersection_area(a, b), 0.0f);
	EXPECT_FLOAT_EQ(box_intersection_area(b, a), 0.0f);
}

TEST(BoxIntersection, DisjointOnASingleAxisIsStillZero)
{
	// The case aabb::surface_area() would get wrong: overlapping on x and z but
	// separated on y. A naive min/max intersection has a negative y extent, and
	// surface_area only guards min.x > max.x, so it would return a bogus
	// (possibly negative) value.
	const aabb a(vec3(0.0f, 0.0f, 0.0f), vec3(4.0f, 1.0f, 4.0f));
	const aabb b(vec3(0.0f, 3.0f, 0.0f), vec3(4.0f, 4.0f, 4.0f));
	EXPECT_FLOAT_EQ(box_intersection_area(a, b), 0.0f);
}

TEST(BoxIntersection, TouchingBoxesHaveZeroVolumeButAFlatIntersection)
{
	// Sharing exactly one face: the intersection is a 0 x 4 x 4 slab, whose
	// surface area is 2*(0*4 + 4*4 + 4*0) = 32.
	const aabb a(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 4.0f, 4.0f));
	const aabb b(vec3(2.0f, 0.0f, 0.0f), vec3(4.0f, 4.0f, 4.0f));
	EXPECT_FLOAT_EQ(box_intersection_area(a, b), 32.0f);
}

TEST(BoxIntersection, PartialOverlap)
{
	// Overlap region is [1,2] x [0,4] x [0,4] -> 1 x 4 x 4.
	// SA = 2*(1*4 + 4*4 + 4*1) = 2*24 = 48.
	const aabb a(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 4.0f, 4.0f));
	const aabb b(vec3(1.0f, 0.0f, 0.0f), vec3(4.0f, 4.0f, 4.0f));
	EXPECT_FLOAT_EQ(box_intersection_area(a, b), 48.0f);
}

TEST(BoxIntersection, FullContainmentGivesTheSmallerBoxArea)
{
	const aabb outer(vec3(0.0f), vec3(4.0f));
	const aabb inner(vec3(1.0f), vec3(2.0f));
	EXPECT_FLOAT_EQ(box_intersection_area(outer, inner), inner.surface_area());
}

TEST(BoxIntersection, IdenticalBoxesGiveTheirOwnArea)
{
	const aabb a(vec3(0.0f), vec3(2.0f, 4.0f, 4.0f));
	EXPECT_FLOAT_EQ(box_intersection_area(a, a), a.surface_area());
}

// ---------------------------------------------------------------------------
// initialization -- the actual regression
// ---------------------------------------------------------------------------

// Deterministically fails without the `{}` fix: an empty tree touches no
// bucket, so every counter must still read exactly zero. With the old
// declaration these are indeterminate.
TEST(OverlapProfile, EveryBucketIsZeroInitialized)
{
	overlap_profile p;

	EXPECT_EQ(p.depth_count, 0u);
	EXPECT_EQ(p.nodes_beyond_buckets, 0u);

	for (u32 d = 0; d < overlap_profile::max_depth_buckets; ++d)
	{
		ASSERT_EQ(p.depth[d].internal_nodes, 0u) << "depth " << d;
		ASSERT_EQ(p.depth[d].pair_count, 0ull) << "depth " << d;
		ASSERT_DOUBLE_EQ(p.depth[d].mean_child_area_ratio, 0.0) << "depth " << d;
		ASSERT_DOUBLE_EQ(p.depth[d].mean_pair_overlap, 0.0) << "depth " << d;
		ASSERT_DOUBLE_EQ(p.depth[d].p95_pair_overlap, 0.0) << "depth " << d;
		ASSERT_DOUBLE_EQ(p.depth[d].max_pair_overlap, 0.0) << "depth " << d;
		ASSERT_DOUBLE_EQ(p.depth[d].sum_pair_overlap, 0.0) << "depth " << d;
	}
}

TEST(OverlapProfile, EveryBucketStaysZeroForAnEmptyTree)
{
	bvh2 empty;
	const overlap_profile p = compute_overlap_profile(empty);

	EXPECT_EQ(p.depth_count, 0u);
	for (u32 d = 0; d < overlap_profile::max_depth_buckets; ++d)
		ASSERT_EQ(p.depth[d].internal_nodes, 0u) << "depth " << d;
}

// ---------------------------------------------------------------------------
// exact depth-bucket counts
// ---------------------------------------------------------------------------

TEST(OverlapProfile, RootIsTheOnlyNodeInDepthZero)
{
	hand_tree h;
	h.build(0.0f);

	const overlap_profile p = compute_overlap_profile(h.tree);

	// Exactly one internal node at depth 0: the root. Not "at least one".
	EXPECT_EQ(p.depth[0].internal_nodes, 1u);
	EXPECT_EQ(p.depth[0].pair_count, 1ull) << "one pair for a binary root";
}

TEST(OverlapProfile, EveryPopulatedBucketIsAccountedFor)
{
	mesh m;
	{
		const char* roots[] = {"scenes/", "../scenes/", "../../scenes/", "../../../scenes/"};
		bool loaded = false;
		for (const char* root : roots)
			if (m.load_obj(std::string(root) + "teapot.obj")) { loaded = true; break; }
		if (!loaded) GTEST_SKIP() << "teapot.obj not found";
	}

	bvh2 tree;
	build_args a; a.silent = true;
	tree.build(m, a);
	tree.apply_reorder(m);

	const overlap_profile p = compute_overlap_profile(tree);

	// The sum over every bucket must equal the tree's interior-node count, with
	// nothing lost past the bucket limit. Checking only a few depths -- as the
	// old three-column CSV did -- cannot catch a bucketing error.
	u64 total = 0;
	for (u32 d = 0; d < overlap_profile::max_depth_buckets; ++d)
		total += p.depth[d].internal_nodes;
	total += p.nodes_beyond_buckets;

	EXPECT_EQ(total, u64(tree.report().interior_count));

	// Every bucket below depth_count that has nodes must have consistent
	// derived values; every bucket at or above it must be untouched.
	for (u32 d = p.depth_count; d < overlap_profile::max_depth_buckets; ++d)
		ASSERT_EQ(p.depth[d].internal_nodes, 0u) << "bucket " << d << " past depth_count";
}

// ---------------------------------------------------------------------------
// the two metrics are different things
// ---------------------------------------------------------------------------

// THE headline distinction. Two disjoint children still expand the surface
// area, so the old "mean_overlap" reported a large value for a tree with
// literally zero overlap.
TEST(OverlapProfile, DisjointChildrenExpandAreaButDoNotOverlap)
{
	hand_tree h;
	h.build(0.0f); // children touch at x=2, no interior overlap

	const overlap_profile p = compute_overlap_profile(h.tree);
	const depth_overlap_stats& s = p.depth[0];

	ASSERT_EQ(s.internal_nodes, 1u);

	// Parent 4x4x4  -> SA = 2*(16+16+16) = 96.
	// Each child 2x4x4 -> SA = 2*(8+16+8) = 64. Two of them = 128.
	// ratio = 128/96 = 4/3.
	EXPECT_NEAR(s.mean_child_area_ratio, 4.0 / 3.0, 1e-6)
	    << "area expansion is large even though the children are disjoint";

	// The intersection is a degenerate 0x4x4 slab: zero volume, but SA 32.
	// 32/96 = 1/3. This is the shared-face case, and it is exactly why the
	// metric must be documented: a flat intersection is not a volume overlap.
	EXPECT_NEAR(s.mean_pair_overlap, 32.0 / 96.0, 1e-6);
}

TEST(OverlapProfile, FullyCoincidentChildrenMaximizePairOverlap)
{
	hand_tree h;
	h.build(2.0f); // second child slid fully onto the first

	const overlap_profile p = compute_overlap_profile(h.tree);
	const depth_overlap_stats& s = p.depth[0];

	// Children now both span x in [0,4] and [0,2] respectively... the shifted
	// child is [0,4]x[0,4]x[0,4] intersected with the first [0,2]x[0,4]x[0,4],
	// so the intersection is the whole first child: SA 64. 64/96 = 2/3.
	EXPECT_NEAR(s.mean_pair_overlap, 64.0 / 96.0, 1e-6);
	EXPECT_GT(s.mean_pair_overlap, 32.0 / 96.0) << "more overlap than the touching case";
}

TEST(OverlapProfile, PairOverlapRisesMonotonicallyWithActualOverlap)
{
	double previous = -1.0;
	for (f32 shift : {0.0f, 0.5f, 1.0f, 1.5f, 2.0f})
	{
		hand_tree h;
		h.build(shift);
		const double v = compute_overlap_profile(h.tree).depth[0].mean_pair_overlap;
		EXPECT_GT(v, previous) << "shift " << shift;
		previous = v;
	}
}

TEST(OverlapProfile, MaxAndP95TrackTheSamples)
{
	hand_tree h;
	h.build(1.0f);

	const depth_overlap_stats& s = compute_overlap_profile(h.tree).depth[0];

	// One pair, so mean, p95 and max are all the same sample.
	EXPECT_EQ(s.pair_count, 1ull);
	EXPECT_NEAR(s.max_pair_overlap, s.mean_pair_overlap, 1e-9);
	EXPECT_NEAR(s.p95_pair_overlap, s.mean_pair_overlap, 1e-9);
}

// ---------------------------------------------------------------------------
// normalization across widths
// ---------------------------------------------------------------------------

// The reason mean and sum are reported separately. C(n,2) is 1 at width 2, 6 at
// width 4, 28 at width 8, so the SUM grows with width for structural reasons
// that have nothing to do with geometry. Only the mean is comparable.
TEST(OverlapProfile, PairCountFollowsTheBinomialAndOnlyTheMeanIsComparable)
{
	mesh m;
	{
		const char* roots[] = {"scenes/", "../scenes/", "../../scenes/", "../../../scenes/"};
		bool loaded = false;
		for (const char* root : roots)
			if (m.load_obj(std::string(root) + "teapot.obj")) { loaded = true; break; }
		if (!loaded) GTEST_SKIP() << "teapot.obj not found";
	}

	for (u32 width : {2u, 4u, 8u})
	{
		mesh mm = m;
		bvh2 tree;
		build_args a; a.silent = true;
		tree.build(mm, a);
		tree.apply_reorder(mm);

		if (width > 2)
		{
			collapse_args ca;
			ca.width = width;
			ca.silent = true;
			collapse(tree, mm, ca);
		}

		const overlap_profile p = compute_overlap_profile(tree);

		// pair_count must equal sum over internal nodes of C(child_cnt, 2).
		u64 expected = 0;
		std::vector<u32> depth(tree.nodes().size(), 0u);
		std::vector<u64> per_depth(overlap_profile::max_depth_buckets, 0ull);

		for (u32 i = 0; i < tree.nodes().size(); ++i)
		{
			const bvh2_node& node = tree.nodes()[i];
			const u32 d = depth[i];
			if (!node.ptr.is_int) continue;
			for (u32 c = 0; c < node.ptr.child_cnt; ++c)
				depth[node.ptr.child_idx + c] = d + 1;
			if (d >= overlap_profile::max_depth_buckets) continue;
			const u64 n = node.ptr.child_cnt;
			per_depth[d] += n * (n - 1) / 2;
			expected += n * (n - 1) / 2;
		}

		u64 got = 0;
		for (u32 d = 0; d < overlap_profile::max_depth_buckets; ++d)
		{
			ASSERT_EQ(p.depth[d].pair_count, per_depth[d]) << "width " << width << " depth " << d;
			got += p.depth[d].pair_count;
		}
		EXPECT_EQ(got, expected) << "width " << width;

		// And the mean must be the sum divided by that count, exactly.
		for (u32 d = 0; d < p.depth_count; ++d)
		{
			const depth_overlap_stats& s = p.depth[d];
			if (s.pair_count == 0) continue;
			ASSERT_NEAR(s.mean_pair_overlap, s.sum_pair_overlap / double(s.pair_count), 1e-12)
			    << "width " << width << " depth " << d;
		}
	}
}
