#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <core/rng.h>
#include <eval/directional_geometry.h>
#include <util/mesh.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
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

	void finish(mesh& m)
	{
		const u32 n = m.triangle_count();
		m.normal_indices.assign(n, uvec3(invalid_id, invalid_id, invalid_id));
		m.tex_coord_indices.assign(n, uvec3(invalid_id, invalid_id, invalid_id));
		m.material_indices.assign(n, invalid_id);
		m.compute_bounds();
	}

	// One triangle in each coordinate plane, each with projected area 0.5.
	mesh axis_plane_triangles()
	{
		mesh m;
		m.vertices = {
			vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), // XY
			vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 0.0f, 1.0f), // XZ
			vec3(0.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 0.0f, 1.0f), // YZ
		};
		m.vertex_indices = { uvec3(0, 1, 2), uvec3(3, 4, 5), uvec3(6, 7, 8) };
		finish(m);
		return m;
	}

	// Two coincident unit quads in the plane z == 0. Their projections overlap
	// exactly, so the sum of projected areas is twice the projected box area.
	mesh coincident_quads()
	{
		mesh m;
		m.vertices = {
			vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f),
		};
		m.vertex_indices = {
			uvec3(0, 1, 2), uvec3(0, 2, 3),
			uvec3(0, 1, 2), uvec3(0, 2, 3),
		};
		finish(m);
		return m;
	}

	// Four triangles at two depths: a small hand-checkable tree.
	mesh two_quads()
	{
		mesh m;
		m.vertices = {
			vec3(-1.0f, -1.0f, 1.0f), vec3(1.0f, -1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f), vec3(-1.0f, 1.0f, 1.0f),
			vec3(-1.0f, -1.0f, 5.0f), vec3(1.0f, -1.0f, 5.0f), vec3(1.0f, 1.0f, 5.0f), vec3(-1.0f, 1.0f, 5.0f),
		};
		m.vertex_indices = { uvec3(0, 1, 2), uvec3(0, 2, 3), uvec3(4, 5, 6), uvec3(4, 6, 7) };
		finish(m);
		return m;
	}

	mesh random_triangles(u32 count, u64 seed)
	{
		mesh m;
		m.vertices.reserve(count * 3);
		m.vertex_indices.reserve(count);

		rng random(seed, 1u);
		for (u32 i = 0; i < count; ++i)
		{
			for (u32 v = 0; v < 3; ++v)
				m.vertices.push_back(vec3(random.next_f32() * 4.0f - 2.0f,
					random.next_f32() * 4.0f - 2.0f,
					random.next_f32() * 4.0f - 2.0f));
			m.vertex_indices.push_back(uvec3(i * 3 + 0, i * 3 + 1, i * 3 + 2));
		}
		finish(m);
		return m;
	}

	directional_geometry direct_sum(const mesh& m)
	{
		directional_geometry g;
		for (u32 i = 0; i < m.triangle_count(); ++i)
			g.add(triangle_directional_geometry(m.get_triangle(i)));
		return g;
	}

	// Field-exact comparison. memcmp would also compare the trailing padding
	// bytes of directional_geometry, which no operation is required to define.
	bool bitwise_equal(const std::vector<directional_geometry>& a,
		const std::vector<directional_geometry>& b)
	{
		if (a.size() != b.size()) return false;
		for (size_t i = 0; i < a.size(); ++i)
		{
			if (a[i].p_yz != b[i].p_yz) return false;
			if (a[i].p_xz != b[i].p_xz) return false;
			if (a[i].p_xy != b[i].p_xy) return false;
			if (a[i].triangle_surface_area_sum != b[i].triangle_surface_area_sum) return false;
			if (a[i].primitive_count != b[i].primitive_count) return false;
		}
		return true;
	}

	directional_ratio make_ratio(double raw)
	{
		directional_ratio r;
		r.raw = raw;
		r.clamped = raw < 0.0 ? 0.0 : (raw > 1.0 ? 1.0 : raw);
		r.emptiness_lower = 1.0 - r.clamped;
		r.valid = true;
		return r;
	}

	constexpr double sqrt_half = 0.70710678118654752440;

} // namespace

// ---------------------------------------------------------------- algebra

TEST(DirectionalGeometry, AxisPlaneTrianglesMapToTheExpectedComponent)
{
	const mesh m = axis_plane_triangles();

	const directional_geometry xy = triangle_directional_geometry(m.get_triangle(0));
	EXPECT_DOUBLE_EQ(xy.p_xy, 0.5);
	EXPECT_DOUBLE_EQ(xy.p_xz, 0.0);
	EXPECT_DOUBLE_EQ(xy.p_yz, 0.0);
	EXPECT_DOUBLE_EQ(xy.triangle_surface_area_sum, 0.5);
	EXPECT_EQ(xy.primitive_count, 1u);

	const directional_geometry xz = triangle_directional_geometry(m.get_triangle(1));
	EXPECT_DOUBLE_EQ(xz.p_xz, 0.5);
	EXPECT_DOUBLE_EQ(xz.p_xy, 0.0);
	EXPECT_DOUBLE_EQ(xz.p_yz, 0.0);

	const directional_geometry yz = triangle_directional_geometry(m.get_triangle(2));
	EXPECT_DOUBLE_EQ(yz.p_yz, 0.5);
	EXPECT_DOUBLE_EQ(yz.p_xy, 0.0);
	EXPECT_DOUBLE_EQ(yz.p_xz, 0.0);
}

TEST(DirectionalGeometry, ZeroAreaTrianglesContributeZeroWithoutNaNs)
{
	const triangle degenerate(vec3(1.0f, 2.0f, 3.0f), vec3(1.0f, 2.0f, 3.0f), vec3(1.0f, 2.0f, 3.0f));
	const directional_geometry a = triangle_directional_geometry(degenerate);
	EXPECT_DOUBLE_EQ(a.p_yz, 0.0);
	EXPECT_DOUBLE_EQ(a.p_xz, 0.0);
	EXPECT_DOUBLE_EQ(a.p_xy, 0.0);
	EXPECT_DOUBLE_EQ(a.triangle_surface_area_sum, 0.0);

	// Collinear vertices: also zero area, still no NaN.
	const triangle collinear(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, 1.0f, 1.0f), vec3(2.0f, 2.0f, 2.0f));
	const directional_geometry b = triangle_directional_geometry(collinear);
	EXPECT_DOUBLE_EQ(b.p_yz, 0.0);
	EXPECT_DOUBLE_EQ(b.p_xz, 0.0);
	EXPECT_DOUBLE_EQ(b.p_xy, 0.0);
	EXPECT_FALSE(std::isnan(b.triangle_surface_area_sum));

	const unit_direction u = normalize_direction(vec3(0.3f, -0.5f, 0.8f));
	EXPECT_DOUBLE_EQ(axis_projected_area_upper(b, u), 0.0);
	EXPECT_DOUBLE_EQ(triangle_exact_projected_area(collinear, u), 0.0);
}

TEST(DirectionalGeometry, DirectionScaleDoesNotChangeTheNormalizedResult)
{
	const vec3 d(0.3f, -0.5f, 0.8f);
	const unit_direction a = normalize_direction(d);
	ASSERT_TRUE(a.valid);
	EXPECT_NEAR(a.x * a.x + a.y * a.y + a.z * a.z, 1.0, 1e-15);

	// Powers of two scale an f32 vector exactly, so only the descriptor's own
	// double arithmetic is under test here.
	for (f32 s : {4.0f, 0.25f, 1024.0f, 1.0f / 4096.0f})
	{
		const unit_direction b = normalize_direction(d * s);
		ASSERT_TRUE(b.valid) << "scale " << s;
		EXPECT_NEAR(a.x, b.x, 1e-15) << "scale " << s;
		EXPECT_NEAR(a.y, b.y, 1e-15) << "scale " << s;
		EXPECT_NEAR(a.z, b.z, 1e-15) << "scale " << s;
	}

	// A scale that is not a power of two rounds the f32 input itself, so the
	// agreement is bounded by f32 precision, not by the descriptor.
	for (f32 s : {3.5f, 1e-6f, 7.25e3f})
	{
		const unit_direction b = normalize_direction(d * s);
		ASSERT_TRUE(b.valid) << "scale " << s;
		EXPECT_NEAR(a.x, b.x, 1e-6) << "scale " << s;
		EXPECT_NEAR(a.y, b.y, 1e-6) << "scale " << s;
		EXPECT_NEAR(a.z, b.z, 1e-6) << "scale " << s;
	}

	const mesh m = random_triangles(16, 0x51ull);
	const directional_geometry g = direct_sum(m);

	const aabb box = m.bounds();
	const directional_ratio ra = compute_directional_ratio(g, box, d);
	const directional_ratio rb = compute_directional_ratio(g, box, d * 4.0f);
	const directional_ratio rc = compute_directional_ratio(g, box, d * 3.5f);
	ASSERT_TRUE(ra.valid);
	ASSERT_TRUE(rb.valid);
	ASSERT_TRUE(rc.valid);
	EXPECT_NEAR(ra.raw, rb.raw, 1e-15);
	EXPECT_NEAR(ra.raw, rc.raw, 1e-6);
}

TEST(DirectionalGeometry, UpperBoundEqualsExactOnCoordinateAxisDirections)
{
	const mesh m = random_triangles(64, 0xa17ull);
	const directional_geometry g = direct_sum(m);

	const vec3 axes[3] = { vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), vec3(0.0f, 0.0f, 1.0f) };

	for (const vec3& axis : axes)
	{
		const unit_direction u = normalize_direction(axis);
		ASSERT_TRUE(u.valid);

		double exact = 0.0;
		for (u32 i = 0; i < m.triangle_count(); ++i)
			exact += triangle_exact_projected_area(m.get_triangle(i), u);

		EXPECT_DOUBLE_EQ(axis_projected_area_upper(g, u), exact);
	}
}

TEST(DirectionalGeometry, UpperBoundDominatesExactForRandomTrianglesAndDirections)
{
	const mesh m = random_triangles(256, 0xbeefull);
	const directional_geometry g = direct_sum(m);

	rng random(0xf00dull, 3u);
	for (u32 trial = 0; trial < 256; ++trial)
	{
		const vec3 d(random.next_f32() * 2.0f - 1.0f,
			random.next_f32() * 2.0f - 1.0f,
			random.next_f32() * 2.0f - 1.0f);

		const unit_direction u = normalize_direction(d);
		if (!u.valid) continue;

		double exact = 0.0;
		for (u32 i = 0; i < m.triangle_count(); ++i)
			exact += triangle_exact_projected_area(m.get_triangle(i), u);

		const double upper = axis_projected_area_upper(g, u);

		// The triangle inequality is exact in real arithmetic; allow a relative
		// slack of one part in 1e12 for the double summation.
		EXPECT_GE(upper, exact - 1e-12 * std::max(1.0, exact));
	}
}

TEST(DirectionalGeometry, SignCancellationMakesTheBoundLoose)
{
	// Normal along (1,1,0); queried along (1,-1,0), which is orthogonal to it.
	const triangle t(vec3(0.0f, 0.0f, 0.0f), vec3(1.0f, -1.0f, 0.0f), vec3(0.0f, 0.0f, 1.0f));

	const unit_direction u = normalize_direction(vec3(1.0f, -1.0f, 0.0f));
	ASSERT_TRUE(u.valid);

	EXPECT_NEAR(triangle_exact_projected_area(t, u), 0.0, 1e-15);

	const directional_geometry g = triangle_directional_geometry(t);
	EXPECT_NEAR(axis_projected_area_upper(g, u), sqrt_half, 1e-12);
	EXPECT_GT(axis_projected_area_upper(g, u), 0.0);
}

// ------------------------------------------------------------- saturation

TEST(DirectionalGeometry, CoincidentQuadsSaturateTheRatio)
{
	const mesh m = coincident_quads();
	const directional_geometry g = direct_sum(m);

	// Two coincident unit quads: 2.0 of projected XY area inside a 1x1 face.
	EXPECT_DOUBLE_EQ(g.p_xy, 2.0);
	EXPECT_EQ(g.primitive_count, 4u);

	const aabb box = m.bounds();
	const directional_ratio r = compute_directional_ratio(g, box, vec3(0.0f, 0.0f, 1.0f));

	ASSERT_TRUE(r.valid);
	EXPECT_DOUBLE_EQ(r.raw, 2.0);
	EXPECT_DOUBLE_EQ(r.clamped, 1.0);
	EXPECT_DOUBLE_EQ(r.emptiness_lower, 0.0);
	EXPECT_EQ(classify_ratio(r), ratio_bin::raw_gt_1);
}

TEST(DirectionalGeometry, SingleTriangleNeverExceedsItsOwnBox)
{
	const mesh m = random_triangles(128, 0xc0ffeeull);

	rng random(0x1234ull, 7u);
	for (u32 i = 0; i < m.triangle_count(); ++i)
	{
		const triangle t = m.get_triangle(i);
		const directional_geometry g = triangle_directional_geometry(t);
		const aabb box = t.bounds();

		for (u32 trial = 0; trial < 8; ++trial)
		{
			const vec3 d(random.next_f32() * 2.0f - 1.0f,
				random.next_f32() * 2.0f - 1.0f,
				random.next_f32() * 2.0f - 1.0f);

			const directional_ratio r = compute_directional_ratio(g, box, d);
			if (!r.valid) continue;
			EXPECT_LE(r.raw, 1.0 + 1e-9);
		}
	}
}

// -------------------------------------------------------------- degeneracy

TEST(DirectionalGeometry, InvalidDirectionsReturnInvalid)
{
	const mesh m = two_quads();
	const directional_geometry g = direct_sum(m);
	const aabb box = m.bounds();

	const vec3 bad[] = {
		vec3(0.0f, 0.0f, 0.0f),
		vec3(std::numeric_limits<f32>::quiet_NaN(), 1.0f, 0.0f),
		vec3(std::numeric_limits<f32>::infinity(), 0.0f, 0.0f),
		vec3(0.0f, -std::numeric_limits<f32>::infinity(), 0.0f),
	};

	for (const vec3& d : bad)
	{
		EXPECT_FALSE(normalize_direction(d).valid);

		const directional_ratio r = compute_directional_ratio(g, box, d);
		EXPECT_FALSE(r.valid);
		EXPECT_TRUE(std::isnan(r.raw));
		EXPECT_TRUE(std::isnan(r.clamped));
		EXPECT_TRUE(std::isnan(r.emptiness_lower));
		EXPECT_EQ(classify_ratio(r), ratio_bin::invalid);

		EXPECT_TRUE(std::isnan(axis_projected_area_upper(g, d)));
		EXPECT_TRUE(std::isnan(projected_box_area(box, d)));
	}
}

TEST(DirectionalGeometry, DegenerateBoxesReturnInvalid)
{
	const mesh m = coincident_quads();
	const directional_geometry g = direct_sum(m);

	// Flat box (ez == 0) queried along a direction inside its own plane:
	// B_box collapses to zero.
	const aabb flat = m.bounds();
	EXPECT_DOUBLE_EQ(projected_box_area(flat, vec3(1.0f, 0.0f, 0.0f)), 0.0);

	const directional_ratio r = compute_directional_ratio(g, flat, vec3(1.0f, 0.0f, 0.0f));
	EXPECT_FALSE(r.valid);
	EXPECT_EQ(classify_ratio(r), ratio_bin::invalid);

	// A point box has no projected area in any direction.
	const aabb point(vec3(1.0f, 1.0f, 1.0f), vec3(1.0f, 1.0f, 1.0f));
	EXPECT_FALSE(compute_directional_ratio(g, point, vec3(0.3f, 0.4f, 0.5f)).valid);

	// An empty (default-constructed) box is treated as having no extent.
	const aabb empty;
	EXPECT_TRUE(empty.empty());
	EXPECT_DOUBLE_EQ(projected_box_area(empty, vec3(0.3f, 0.4f, 0.5f)), 0.0);
	EXPECT_FALSE(compute_directional_ratio(g, empty, vec3(0.3f, 0.4f, 0.5f)).valid);
}

TEST(DirectionalGeometry, NoInvalidRatioEverLandsInAValidBin)
{
	directional_ratio invalid;
	EXPECT_FALSE(invalid.valid);
	EXPECT_EQ(classify_ratio(invalid), ratio_bin::invalid);

	directional_ratio nan_but_flagged_valid;
	nan_but_flagged_valid.valid = true;
	nan_but_flagged_valid.raw = std::numeric_limits<double>::quiet_NaN();
	EXPECT_EQ(classify_ratio(nan_but_flagged_valid), ratio_bin::invalid);
}

TEST(DirectionalGeometry, RatioBinBoundariesAreTheFrozenOnes)
{
	EXPECT_EQ(classify_ratio(make_ratio(0.0)), ratio_bin::q_00_10);
	EXPECT_EQ(classify_ratio(make_ratio(0.0999)), ratio_bin::q_00_10);
	EXPECT_EQ(classify_ratio(make_ratio(0.1)), ratio_bin::q_10_20);
	EXPECT_EQ(classify_ratio(make_ratio(0.55)), ratio_bin::q_50_60);
	EXPECT_EQ(classify_ratio(make_ratio(0.8999)), ratio_bin::q_80_90);
	EXPECT_EQ(classify_ratio(make_ratio(0.9)), ratio_bin::q_90_100);
	EXPECT_EQ(classify_ratio(make_ratio(1.0)), ratio_bin::q_90_100);
	EXPECT_EQ(classify_ratio(make_ratio(1.0 + 1e-12)), ratio_bin::raw_gt_1);
	EXPECT_EQ(classify_ratio(make_ratio(7.5)), ratio_bin::raw_gt_1);

	EXPECT_EQ(ratio_bin_count, 12u);
	EXPECT_STREQ(to_string(ratio_bin::q_00_10), "q_00_10");
	EXPECT_STREQ(to_string(ratio_bin::raw_gt_1), "raw_gt_1");
	EXPECT_STREQ(to_string(ratio_bin::invalid), "invalid");
}

// -------------------------------------------------------------- additivity

TEST(DirectionalGeometry, HandBuiltTreeIsAdditive)
{
	mesh m = two_quads();

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.max_leaf_size = 1;
	ba.silent = true;

	bvh2 tree;
	tree.build(m, ba);

	const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);
	ASSERT_EQ(g.size(), tree.nodes().size());

	// Every quad is 2 unit-normal triangles of area 2 in the plane z = const.
	const directional_geometry total = direct_sum(m);
	EXPECT_DOUBLE_EQ(total.p_xy, 8.0);
	EXPECT_DOUBLE_EQ(total.p_yz, 0.0);
	EXPECT_DOUBLE_EQ(total.p_xz, 0.0);

	EXPECT_DOUBLE_EQ(g[0].p_xy, total.p_xy);
	EXPECT_DOUBLE_EQ(g[0].triangle_surface_area_sum, total.triangle_surface_area_sum);
	EXPECT_EQ(g[0].primitive_count, m.triangle_count());

	for (u32 i = 0; i < tree.nodes().size(); ++i)
	{
		const bvh2_node& node = tree.nodes()[i];
		if (!node.ptr.is_int) continue;

		directional_geometry sum;
		for (u32 c = 0; c < node.ptr.child_cnt; ++c) sum.add(g[node.ptr.child_idx + c]);

		EXPECT_DOUBLE_EQ(g[i].p_yz, sum.p_yz);
		EXPECT_DOUBLE_EQ(g[i].p_xz, sum.p_xz);
		EXPECT_DOUBLE_EQ(g[i].p_xy, sum.p_xy);
		EXPECT_DOUBLE_EQ(g[i].triangle_surface_area_sum, sum.triangle_surface_area_sum);
		EXPECT_EQ(g[i].primitive_count, sum.primitive_count);
	}
}

TEST(DirectionalGeometry, AdditiveOnBVH2AndWideTrees)
{
	mesh original;
	const std::string path = find_scene("teapot.obj");
	ASSERT_FALSE(path.empty()) << "teapot.obj not found";
	ASSERT_TRUE(original.load_obj(path));

	for (u32 width : {2u, 4u, 8u})
	{
		mesh m = original;

		build_args ba;
		ba.method = split_method::binned_sah;
		ba.max_leaf_size = 1;
		ba.silent = true;

		bvh2 tree;
		tree.build(m, ba);
		tree.apply_reorder(m);
		tree.refit(m);

		if (width > 2)
		{
			collapse_args ca;
			ca.width = width;
			ca.method = collapse_method::dynamic_programming;
			ca.silent = true;
			collapse(tree, m, ca);
		}

		const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);
		ASSERT_EQ(g.size(), tree.nodes().size());

		const directional_geometry total = direct_sum(m);
		EXPECT_EQ(g[0].primitive_count, m.triangle_count()) << "width " << width;
		EXPECT_NEAR(g[0].p_yz, total.p_yz, 1e-9 * std::max(1.0, total.p_yz)) << "width " << width;
		EXPECT_NEAR(g[0].p_xz, total.p_xz, 1e-9 * std::max(1.0, total.p_xz)) << "width " << width;
		EXPECT_NEAR(g[0].p_xy, total.p_xy, 1e-9 * std::max(1.0, total.p_xy)) << "width " << width;

		u32 internal = 0;
		for (u32 i = 0; i < tree.nodes().size(); ++i)
		{
			const bvh2_node& node = tree.nodes()[i];
			if (!node.ptr.is_int) continue;
			++internal;

			directional_geometry sum;
			for (u32 c = 0; c < node.ptr.child_cnt; ++c) sum.add(g[node.ptr.child_idx + c]);

			EXPECT_DOUBLE_EQ(g[i].p_yz, sum.p_yz);
			EXPECT_DOUBLE_EQ(g[i].p_xz, sum.p_xz);
			EXPECT_DOUBLE_EQ(g[i].p_xy, sum.p_xy);
			EXPECT_DOUBLE_EQ(g[i].triangle_surface_area_sum, sum.triangle_surface_area_sum);
			EXPECT_EQ(g[i].primitive_count, sum.primitive_count);
		}
		EXPECT_GT(internal, 0u) << "width " << width;
	}
}

TEST(DirectionalGeometry, IdenticalBeforeAndAfterApplyReorder)
{
	mesh original;
	const std::string path = find_scene("teapot.obj");
	ASSERT_FALSE(path.empty()) << "teapot.obj not found";
	ASSERT_TRUE(original.load_obj(path));

	mesh m = original;

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.max_leaf_size = 1;
	ba.silent = true;

	bvh2 tree;
	tree.build(m, ba);

	// build() already refits against the un-permuted mesh; the descriptor must
	// go through tree.prim_index() to be correct here.
	const std::vector<directional_geometry> before = compute_directional_geometry(tree, m);

	tree.apply_reorder(m);
	tree.refit(m);

	const std::vector<directional_geometry> after = compute_directional_geometry(tree, m);

	EXPECT_TRUE(bitwise_equal(before, after));
}

TEST(DirectionalGeometry, DeterministicForTheSameTreeAndMesh)
{
	mesh m;
	const std::string path = find_scene("teapot.obj");
	ASSERT_FALSE(path.empty()) << "teapot.obj not found";
	ASSERT_TRUE(m.load_obj(path));

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.max_leaf_size = 1;
	ba.silent = true;

	bvh2 tree;
	tree.build(m, ba);
	tree.apply_reorder(m);
	tree.refit(m);

	const std::vector<directional_geometry> a = compute_directional_geometry(tree, m);
	const std::vector<directional_geometry> b = compute_directional_geometry(tree, m);

	EXPECT_TRUE(bitwise_equal(a, b));
}

TEST(DirectionalGeometry, LeafDescriptorMatchesItsOwnPrimitiveSlots)
{
	mesh m;
	const std::string path = find_scene("cornell-box.obj");
	ASSERT_FALSE(path.empty()) << "cornell-box.obj not found";
	ASSERT_TRUE(m.load_obj(path));

	build_args ba;
	ba.method = split_method::binned_sah;
	ba.max_leaf_size = 1;
	ba.silent = true;

	bvh2 tree;
	tree.build(m, ba);
	tree.apply_reorder(m);
	tree.refit(m);

	const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);

	u32 leaves = 0;
	for (u32 i = 0; i < tree.nodes().size(); ++i)
	{
		const bvh2_node& node = tree.nodes()[i];
		if (node.ptr.is_int) continue;
		++leaves;

		directional_geometry sum;
		for (u32 p = 0; p < node.ptr.prim_cnt; ++p)
			sum.add(triangle_directional_geometry(m.get_triangle(tree.prim_index(node.ptr.prim_idx + p))));

		EXPECT_DOUBLE_EQ(g[i].p_yz, sum.p_yz);
		EXPECT_DOUBLE_EQ(g[i].p_xz, sum.p_xz);
		EXPECT_DOUBLE_EQ(g[i].p_xy, sum.p_xy);
		EXPECT_EQ(g[i].primitive_count, sum.primitive_count);
	}
	EXPECT_GT(leaves, 0u);
}
