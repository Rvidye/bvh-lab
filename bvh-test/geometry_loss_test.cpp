#include <build/bvh2_builder.h>
#include <build/collapse.h>
#include <build/geometry_loss.h>
#include <core/traverse_bvh2.h>
#include <eval/trace.h>
#include <eval/directional_geometry.h>
#include <reference/brute_force.h>
#include <util/camera.h>
#include <util/mesh.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <set>
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

	directional_geometry sum_of(const mesh& m)
	{
		directional_geometry g;
		for (u32 i = 0; i < m.triangle_count(); ++i)
			g.add(triangle_directional_geometry(m.get_triangle(i)));
		return g;
	}

	// A unit cube's worth of box, with geometry supplied by the caller.
	aabb unit_box(f32 sx = 1.0f, f32 sy = 1.0f, f32 sz = 1.0f)
	{
		return aabb(vec3(0.0f, 0.0f, 0.0f), vec3(sx, sy, sz));
	}

	void build_tree(bvh2& tree, mesh& m)
	{
		build_args ba;
		ba.method = split_method::binned_sah;
		ba.bins = 32;
		ba.max_leaf_size = 1;
		ba.silent = true;
		tree.build(m, ba);
		tree.apply_reorder(m);
		tree.refit(m);
	}

	// Two axis-aligned quads that fill the XY faces of the box: G is entirely in
	// the XY component.
	mesh xy_slabs()
	{
		mesh m;
		m.vertices = {
			vec3(0,0,0), vec3(1,0,0), vec3(1,1,0), vec3(0,1,0),
			vec3(0,0,1), vec3(1,0,1), vec3(1,1,1), vec3(0,1,1),
		};
		m.vertex_indices = { uvec3(0,1,2), uvec3(0,2,3), uvec3(4,5,6), uvec3(4,6,7) };
		finish(m);
		return m;
	}

	// The same AABB and the same TOTAL triangle area as xy_slabs, but the area is
	// split between the XY and YZ orientations instead of all being XY.
	mesh mixed_slabs()
	{
		mesh m;
		m.vertices = {
			// one unit quad in the XY plane at z = 0        (area 1, normal +/-Z)
			vec3(0,0,0), vec3(1,0,0), vec3(1,1,0), vec3(0,1,0),
			// one unit quad in the YZ plane at x = 0        (area 1, normal +/-X)
			vec3(0,0,0), vec3(0,1,0), vec3(0,1,1), vec3(0,0,1),
			// the two corners that keep the AABB identical to xy_slabs
			vec3(1,1,1), vec3(1,0,1),
		};
		m.vertex_indices = {
			uvec3(0,1,2), uvec3(0,2,3),      // XY quad, total area 1
			uvec3(4,5,6), uvec3(4,6,7),      // YZ quad, total area 1
		};
		// Degenerate sliver carrying no area but pinning the box corner.
		m.vertices.push_back(vec3(1,1,1));
		m.vertex_indices.push_back(uvec3(8, 8, 9));
		finish(m);
		return m;
	}

} // namespace

// ------------------------------------------------------------ descriptor basis

TEST(GeometryLoss, OneTriangleProjectedComponents)
{
	// Right triangle in the XY plane: all projected area lands on XY.
	const triangle xy(vec3(0,0,0), vec3(1,0,0), vec3(0,1,0));
	const directional_geometry g = triangle_directional_geometry(xy);

	EXPECT_DOUBLE_EQ(g.p_xy, 0.5);
	EXPECT_DOUBLE_EQ(g.p_xz, 0.0);
	EXPECT_DOUBLE_EQ(g.p_yz, 0.0);
	EXPECT_DOUBLE_EQ(g.triangle_surface_area_sum, 0.5);

	const triangle yz(vec3(0,0,0), vec3(0,1,0), vec3(0,0,1));
	const directional_geometry gy = triangle_directional_geometry(yz);
	EXPECT_DOUBLE_EQ(gy.p_yz, 0.5);
	EXPECT_DOUBLE_EQ(gy.p_xy, 0.0);
	EXPECT_DOUBLE_EQ(gy.p_xz, 0.0);
}

TEST(GeometryLoss, BottomUpAccumulationMatchesADirectScan)
{
	mesh m;
	const std::string p = find_scene("teapot.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(m.load_obj(p));

	bvh2 tree;
	build_tree(tree, m);

	const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);
	ASSERT_EQ(g.size(), tree.nodes().size());

	const directional_geometry total = sum_of(m);
	EXPECT_EQ(g[0].primitive_count, m.triangle_count());
	EXPECT_NEAR(g[0].p_yz, total.p_yz, 1e-9 * std::max(1.0, total.p_yz));
	EXPECT_NEAR(g[0].p_xz, total.p_xz, 1e-9 * std::max(1.0, total.p_xz));
	EXPECT_NEAR(g[0].p_xy, total.p_xy, 1e-9 * std::max(1.0, total.p_xy));
	EXPECT_NEAR(g[0].triangle_surface_area_sum, total.triangle_surface_area_sum,
		1e-9 * std::max(1.0, total.triangle_surface_area_sum));

	// Every internal node is the sum of its children.
	for (u32 i = 0; i < tree.nodes().size(); ++i)
	{
		const bvh2_node& n = tree.nodes()[i];
		if (!n.ptr.is_int) continue;

		directional_geometry s;
		for (u32 c = 0; c < n.ptr.child_cnt; ++c) s.add(g[n.ptr.child_idx + c]);

		EXPECT_DOUBLE_EQ(g[i].p_yz, s.p_yz);
		EXPECT_DOUBLE_EQ(g[i].p_xz, s.p_xz);
		EXPECT_DOUBLE_EQ(g[i].p_xy, s.p_xy);
		EXPECT_DOUBLE_EQ(g[i].triangle_surface_area_sum, s.triangle_surface_area_sum);
	}
}

// ------------------------------------------------------------------ loss shape

TEST(GeometryLoss, EmptyGeometryGivesFullSurfaceAreaLoss)
{
	const aabb box = unit_box(2.0f, 3.0f, 5.0f);
	const directional_geometry empty;   // G = 0, Atri = 0

	EXPECT_FLOAT_EQ(directional_loss(box, empty), box.surface_area());
	EXPECT_FLOAT_EQ(scalar_density_loss(box, empty), box.surface_area());
}

TEST(GeometryLoss, GeometryCoveringEveryAxisGivesZeroLoss)
{
	const aabb box = unit_box(2.0f, 3.0f, 5.0f);

	// Fx = 15, Fy = 10, Fz = 6. Exceed all three.
	directional_geometry g;
	g.p_yz = 100.0;
	g.p_xz = 100.0;
	g.p_xy = 100.0;
	g.triangle_surface_area_sum = 1000.0;

	EXPECT_FLOAT_EQ(directional_loss(box, g), 0.0f);
	EXPECT_FLOAT_EQ(scalar_density_loss(box, g), 0.0f);
}

TEST(GeometryLoss, LossIsBoundedByZeroAndSurfaceArea)
{
	const aabb box = unit_box(2.0f, 3.0f, 5.0f);
	const f32  sa = box.surface_area();

	for (double s = 0.0; s <= 40.0; s += 1.25)
	{
		directional_geometry g;
		g.p_yz = s; g.p_xz = s; g.p_xy = s;
		g.triangle_surface_area_sum = s;

		const f32 ld = directional_loss(box, g);
		const f32 ls = scalar_density_loss(box, g);

		EXPECT_GE(ld, 0.0f);
		EXPECT_LE(ld, sa);
		EXPECT_GE(ls, 0.0f);
		EXPECT_LE(ls, sa);
	}
}

TEST(GeometryLoss, OrientationSeparatesDirectionalFromScalarControl)
{
	// The whole point of the control: same AABB, same total triangle area,
	// different orientation distribution. Lscalar must agree, Ldir must not.
	mesh a = xy_slabs();
	mesh b = mixed_slabs();

	const directional_geometry ga = sum_of(a);
	const directional_geometry gb = sum_of(b);

	ASSERT_EQ(a.bounds().min, b.bounds().min);
	ASSERT_EQ(a.bounds().max, b.bounds().max);
	ASSERT_NEAR(ga.triangle_surface_area_sum, gb.triangle_surface_area_sum, 1e-9);

	const aabb box = a.bounds();

	EXPECT_FLOAT_EQ(scalar_density_loss(box, ga), scalar_density_loss(box, gb));
	EXPECT_NE(directional_loss(box, ga), directional_loss(box, gb));

	// Concretely: unit box, Fx = Fy = Fz = 1, so SA = 6.
	//   xy_slabs   G = (0, 0, 2)  -> Ldir = 2*((1-0) + (1-0) + 0)      = 4
	//   mixed      G = (1, 0, 1)  -> Ldir = 2*(0 + (1-0) + 0)          = 2
	EXPECT_FLOAT_EQ(directional_loss(box, ga), 4.0f);
	EXPECT_FLOAT_EQ(directional_loss(box, gb), 2.0f);
	EXPECT_FLOAT_EQ(scalar_density_loss(box, ga), 2.0f);   // 6 - 2*2
}

// -------------------------------------------------------------- collapse wiring

TEST(GeometryLoss, MuZeroReproducesOrdinaryCollapseByteForByte)
{
	mesh original;
	const std::string p = find_scene("cornell-box.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(original.load_obj(p));

	mesh m = original;
	bvh2 binary;
	build_tree(binary, m);

	collapse_args ca;
	ca.width = 8;
	ca.method = collapse_method::dynamic_programming;
	ca.max_leaf_size = 1;
	ca.silent = true;

	bvh2 baseline = binary;
	std::vector<u8> emitted_base;
	const collapse_report base = collapse(baseline, m, ca, &emitted_base);

	for (collapse_loss kind : { collapse_loss::directional, collapse_loss::scalar_density })
	{
		const std::vector<f32> area = compute_internal_cost_area(binary, m, { kind, 0.0 });

		bvh2 check = binary;
		collapse_args ca0 = ca;
		ca0.node_internal_area = area.data();
		ca0.node_internal_area_count = static_cast<u32>(area.size());

		std::vector<u8> emitted_zero;
		const collapse_report cr = collapse(check, m, ca0, &emitted_zero);

		EXPECT_EQ(cr.node_count, base.node_count) << to_string(kind);
		EXPECT_EQ(cr.max_depth, base.max_depth) << to_string(kind);
		ASSERT_EQ(check.nodes().size(), baseline.nodes().size()) << to_string(kind);
		EXPECT_EQ(0, std::memcmp(check.nodes().data(), baseline.nodes().data(),
			check.nodes().size() * sizeof(bvh2_node))) << to_string(kind);
		EXPECT_EQ(check.prim_indices(), baseline.prim_indices()) << to_string(kind);
		ASSERT_EQ(emitted_zero.size(), emitted_base.size()) << to_string(kind);
		EXPECT_EQ(0, std::memcmp(emitted_zero.data(), emitted_base.data(), emitted_zero.size()))
			<< to_string(kind);
	}
}

TEST(GeometryLoss, LeafCostPathIsUnchangedByTheLoss)
{
	// The loss enters the internal term only. A tree whose every node is forced
	// to be a leaf by the leaf policy must therefore be identical for any mu.
	mesh original;
	const std::string p = find_scene("cornell-box.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(original.load_obj(p));

	mesh m = original;
	bvh2 binary;
	build_tree(binary, m);

	// A very large internal-cost loss cannot change which primitives a leaf
	// holds, because the leaf term never sees it: every leaf still carries
	// exactly max_leaf_size primitives.
	for (double mu : { 0.0, 2.0, 1000.0 })
	{
		const std::vector<f32> area =
			compute_internal_cost_area(binary, m, { collapse_loss::directional, mu });

		bvh2 t = binary;
		collapse_args ca;
		ca.width = 8;
		ca.method = collapse_method::dynamic_programming;
		ca.max_leaf_size = 1;
		ca.silent = true;
		ca.node_internal_area = area.data();
		ca.node_internal_area_count = static_cast<u32>(area.size());

		collapse(t, m, ca);

		for (const bvh2_node& n : t.nodes())
			if (!n.ptr.is_int) EXPECT_LE(n.ptr.prim_cnt, 1u) << "mu " << mu;
	}
}

// ------------------------------------------------------------------ structural

namespace {

	void check_structure(const bvh2& tree, const mesh& m, u32 width, const char* tag)
	{
		const std::vector<bvh2_node>& nodes = tree.nodes();
		ASSERT_FALSE(nodes.empty()) << tag;

		std::vector<u32> slot_seen(m.triangle_count(), 0u);

		for (u32 i = 0; i < nodes.size(); ++i)
		{
			const bvh2_node& n = nodes[i];
			if (n.ptr.is_int)
			{
				ASSERT_LE(n.ptr.child_cnt, width) << tag << " node " << i;
				ASSERT_GE(n.ptr.child_cnt, 2u) << tag << " node " << i;
				for (u32 c = 0; c < n.ptr.child_cnt; ++c)
				{
					const u32 child = n.ptr.child_idx + c;
					ASSERT_GT(child, i) << tag;             // parents precede children
					ASSERT_LT(child, nodes.size()) << tag;

					// Containment: the parent box holds every child box.
					const aabb& cb = nodes[child].bounds;
					ASSERT_LE(n.bounds.min.x, cb.min.x) << tag;
					ASSERT_LE(n.bounds.min.y, cb.min.y) << tag;
					ASSERT_LE(n.bounds.min.z, cb.min.z) << tag;
					ASSERT_GE(n.bounds.max.x, cb.max.x) << tag;
					ASSERT_GE(n.bounds.max.y, cb.max.y) << tag;
					ASSERT_GE(n.bounds.max.z, cb.max.z) << tag;
				}
			}
			else
			{
				for (u32 s = 0; s < n.ptr.prim_cnt; ++s)
				{
					const u32 slot = n.ptr.prim_idx + s;
					ASSERT_LT(slot, slot_seen.size()) << tag;
					++slot_seen[slot];
				}
			}
		}

		for (u32 s = 0; s < slot_seen.size(); ++s)
			ASSERT_EQ(slot_seen[s], 1u) << tag << " slot " << s;
	}

} // namespace

TEST(GeometryLoss, VariantsPreserveEveryPrimitiveAndValidBounds)
{
	mesh original;
	const std::string p = find_scene("cornell-box.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(original.load_obj(p));

	mesh m = original;
	bvh2 binary;
	build_tree(binary, m);

	for (collapse_loss kind : { collapse_loss::directional, collapse_loss::scalar_density })
	{
		for (double mu : { 0.25, 0.5, 1.0, 2.0 })
		{
			const std::vector<f32> area = compute_internal_cost_area(binary, m, { kind, mu });

			bvh2 t = binary;
			collapse_args ca;
			ca.width = 8;
			ca.method = collapse_method::dynamic_programming;
			ca.max_leaf_size = 1;
			ca.silent = true;
			ca.node_internal_area = area.data();
			ca.node_internal_area_count = static_cast<u32>(area.size());

			const collapse_report cr = collapse(t, m, ca);
			ASSERT_TRUE(cr.stack_bound_ok);

			check_structure(t, m, 8u, to_string(kind));
		}
	}
}

TEST(GeometryLoss, WideTraversalAgreesWithTheBruteForceOracle)
{
	mesh original;
	const std::string p = find_scene("cornell-box.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(original.load_obj(p));

	mesh m = original;
	bvh2 binary;
	build_tree(binary, m);

	const camera cam = camera::frame_bounds(m.bounds(), 48, 48);

	for (collapse_loss kind : { collapse_loss::directional, collapse_loss::scalar_density })
	{
		for (double mu : { 0.5, 1.0, 2.0 })
		{
			const std::vector<f32> area = compute_internal_cost_area(binary, m, { kind, mu });

			bvh2 t = binary;
			collapse_args ca;
			ca.width = 8;
			ca.method = collapse_method::dynamic_programming;
			ca.max_leaf_size = 1;
			ca.silent = true;
			ca.node_internal_area = area.data();
			ca.node_internal_area_count = static_cast<u32>(area.size());
			collapse(t, m, ca);

			const bvh2_view view = t.view();
			const auto      prims = make_prims(m, t);

			std::vector<u32> scratch;
			u64 checked = 0;
			for (u32 j = 0; j < cam.height(); j += 3)
			{
				for (u32 i = 0; i < cam.width(); i += 3)
				{
					const ray r = cam.generate_ray_through_pixel(i, j);

					hit h; h.t = r.t_max;
					null_stats s;
					intersect(view, r, h, prims, s);

					const u32 id = h.valid() ? t.prim_index(h.id) : invalid_id;
					const oracle_result cmp =
						compare_against_oracle(m, r, h.valid(), h.t, id, scratch);
					ASSERT_TRUE(cmp.agree) << to_string(kind) << " mu " << mu
						<< " pixel " << i << "," << j;
					++checked;
				}
			}
			ASSERT_GT(checked, 0u);
		}
	}
}

TEST(GeometryLoss, SummedInternalLossIsFiniteAndOrdered)
{
	mesh original;
	const std::string p = find_scene("teapot.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(original.load_obj(p));

	mesh m = original;
	bvh2 binary;
	build_tree(binary, m);

	const loss_totals t = sum_internal_loss(binary, m);
	EXPECT_GT(t.internal_nodes, 0u);
	EXPECT_TRUE(std::isfinite(t.directional));
	EXPECT_TRUE(std::isfinite(t.scalar_density));
	EXPECT_GE(t.directional, 0.0);
	EXPECT_GE(t.scalar_density, 0.0);
}

// ===================================================================== WP-B
// New per-axis fill variants. The first test is the plan's thesis and was
// written before the implementation.

namespace {

	// Cube with every face area equal to 10, so SA = 60 and the three axes are
	// interchangeable. Any difference between two nodes below is entirely due to
	// how their projected triangle area is distributed across the axes.
	aabb equal_face_box()
	{
		const f32 s = std::sqrt(10.0f);
		return aabb(vec3(0.0f), vec3(s, s, s));
	}

	directional_geometry g_from(double p_yz, double p_xz, double p_xy, double atri)
	{
		directional_geometry g;
		g.p_yz = p_yz;
		g.p_xz = p_xz;
		g.p_xy = p_xy;
		g.triangle_surface_area_sum = atri;
		g.primitive_count = 1;
		return g;
	}

} // namespace

TEST(GeometryLossAniso, SummedLossCannotSeparateAnisotropyButMinCan)
{
	// The defect, stated as a test.
	//
	//   A: G = (0, 10, 10)   per-axis empty [10, 0, 0]   one axis entirely empty
	//   B: G = (20/3 x3)     per-axis empty [10/3 x3]    uniformly loose
	//
	// Both have the same TOTAL empty area, so the summed loss is identical. A is
	// the pathology the idea exists to find; B is a merely loose box that SAH
	// already penalises.
	const aabb box = equal_face_box();

	const directional_geometry a = g_from(0.0, 10.0, 10.0, 20.0);
	const directional_geometry b = g_from(20.0 / 3.0, 20.0 / 3.0, 20.0 / 3.0, 20.0);

	// The current summed loss cannot tell them apart. This documents the defect.
	EXPECT_NEAR(directional_loss(box, a), 20.0f, 1e-3f);
	EXPECT_NEAR(directional_loss(box, b), 20.0f, 1e-3f);
	EXPECT_NEAR(directional_loss(box, a), directional_loss(box, b), 1e-3f);

	// Same total triangle area, so the density control cannot either.
	EXPECT_FLOAT_EQ(scalar_density_loss(box, a), scalar_density_loss(box, b));

	// The worst-axis formulation separates them, and in the right direction:
	// A is worse, so it must carry the larger loss.
	const f32 min_a = directional_min_loss(box, a);
	const f32 min_b = directional_min_loss(box, b);

	EXPECT_GT(min_a, min_b);
	EXPECT_GT(min_a - min_b, 10.0f) << "separation is real, not a rounding artefact";

	// A has an entirely empty axis, so its worst-axis fill is 0 and it takes the
	// whole surface area as loss.
	EXPECT_NEAR(min_a, box.surface_area(), 1e-2f);

	// B's worst axis has q = 2/3, fill = 1 - exp(-2/3) = 0.486583.
	EXPECT_NEAR(min_b, box.surface_area() * (1.0f - 0.486583f), 1e-1f);

	// Spread agrees: A is maximally anisotropic, B is perfectly isotropic.
	EXPECT_GT(directional_spread_loss(box, a), directional_spread_loss(box, b));
	EXPECT_NEAR(directional_spread_loss(box, b), 0.0f, 1e-3f);
}

TEST(GeometryLossAniso, SaturationIsMonotoneUnderTheExponentialAndFlatUnderClamping)
{
	const aabb box = equal_face_box();   // face area 10 per axis

	// q = 2 and q = 10 on every axis.
	const directional_geometry q2 = g_from(20.0, 20.0, 20.0, 30.0);
	const directional_geometry q10 = g_from(100.0, 100.0, 100.0, 150.0);

	// Clamping: both are >= 1 everywhere, so the summed loss is flat at zero and
	// the two are indistinguishable.
	EXPECT_FLOAT_EQ(directional_loss(box, q2), 0.0f);
	EXPECT_FLOAT_EQ(directional_loss(box, q10), 0.0f);

	// The exponential keeps them ordered.
	const f32 min2 = directional_min_loss(box, q2);
	const f32 min10 = directional_min_loss(box, q10);
	EXPECT_GT(min2, min10) << "denser geometry must carry strictly less loss";
	EXPECT_GT(min2, 0.0f);

	// Sanity on the map itself.
	EXPECT_NEAR(saturating_fill(0.0), 0.0, 1e-12);
	EXPECT_NEAR(saturating_fill(1.0), 0.632120558, 1e-9);
	EXPECT_NEAR(saturating_fill(3.0), 0.950212932, 1e-9);
	EXPECT_NEAR(saturating_fill(10.0), 0.999954600, 1e-9);
	EXPECT_LT(saturating_fill(2.0), saturating_fill(10.0));
}

TEST(GeometryLossAniso, DegenerateAxisGuardBehavesAsDocumented)
{
	// A flat plate in XY: ez = 0, so Fx = Fy = 0 and only the XY axis is usable.
	const aabb plate(vec3(0.0f, 0.0f, 0.0f), vec3(2.0f, 3.0f, 0.0f));
	const directional_geometry g = g_from(0.0, 0.0, 3.0, 3.0);

	const axis_fill a = compute_axis_fill(plate, g);
	EXPECT_FALSE(a.valid[0]);
	EXPECT_FALSE(a.valid[1]);
	EXPECT_TRUE(a.valid[2]);
	EXPECT_EQ(a.valid_count, 1u);

	// Fz = 6, G.xy = 3, so q = 0.5 on the only usable axis.
	EXPECT_NEAR(a.q[2], 0.5, 1e-12);

	for (degenerate_axis_policy p : { degenerate_axis_policy::exclude,
									  degenerate_axis_policy::treat_as_full })
	{
		const f32 lmin = directional_min_loss(plate, g, p);
		const f32 lspread = directional_spread_loss(plate, g, p);
		const f32 lsoft = directional_softmin_loss(plate, g, default_softmin_beta, p);

		EXPECT_TRUE(std::isfinite(lmin));
		EXPECT_TRUE(std::isfinite(lspread));
		EXPECT_TRUE(std::isfinite(lsoft));
		EXPECT_GE(lmin, 0.0f);
		EXPECT_LE(lmin, plate.surface_area());
		EXPECT_GE(lspread, 0.0f);
	}

	// exclude: only the XY axis counts, so the min IS that axis.
	EXPECT_NEAR(directional_min_fill(plate, g, degenerate_axis_policy::exclude),
		saturating_fill(0.5), 1e-9);

	// treat_as_full: the two degenerate axes enter as fill 1, which cannot lower
	// a minimum, so the min is unchanged here -- but the SPREAD differs, which is
	// exactly why the two policies are not interchangeable.
	EXPECT_NEAR(directional_min_fill(plate, g, degenerate_axis_policy::treat_as_full),
		saturating_fill(0.5), 1e-9);
	EXPECT_GT(directional_spread_loss(plate, g, degenerate_axis_policy::treat_as_full),
		directional_spread_loss(plate, g, degenerate_axis_policy::exclude));

	// A box with no extent at all has no usable axis under exclude.
	const aabb point(vec3(1.0f), vec3(1.0f));
	EXPECT_FLOAT_EQ(directional_min_loss(point, g, degenerate_axis_policy::exclude), 0.0f);
	EXPECT_FLOAT_EQ(directional_spread_loss(point, g, degenerate_axis_policy::exclude), 0.0f);
}

TEST(GeometryLossAniso, SoftminInterpolatesBetweenMeanAndMin)
{
	const aabb box = equal_face_box();
	const directional_geometry g = g_from(0.0, 10.0, 10.0, 20.0);   // fills 0, .63, .63

	const f32 wide = directional_softmin_loss(box, g, 0.0);       // plain mean
	const f32 tight = directional_softmin_loss(box, g, 200.0);    // effectively min
	const f32 hard = directional_min_loss(box, g);

	EXPECT_LT(wide, tight) << "a larger beta must weight the worst axis more";
	EXPECT_NEAR(tight, hard, 1e-2f * box.surface_area());
}

TEST(GeometryLossAniso, EveryVariantIsByteIdenticalAtMuZero)
{
	mesh original;
	const std::string p = find_scene("cornell-box.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(original.load_obj(p));

	mesh m = original;
	bvh2 binary;
	build_tree(binary, m);

	collapse_args ca;
	ca.width = 8;
	ca.method = collapse_method::dynamic_programming;
	ca.max_leaf_size = 1;
	ca.silent = true;

	bvh2 baseline = binary;
	std::vector<u8> emitted_base;
	const collapse_report base = collapse(baseline, m, ca, &emitted_base);

	for (collapse_loss kind : { collapse_loss::directional,
								collapse_loss::scalar_density,
								collapse_loss::directional_min,
								collapse_loss::directional_softmin,
								collapse_loss::directional_spread })
	{
		geometry_loss_args gla;
		gla.kind = kind;
		gla.mu = 0.0;

		const std::vector<f32> area = compute_internal_cost_area(binary, m, gla);

		bvh2 check = binary;
		collapse_args ca0 = ca;
		ca0.node_internal_area = area.data();
		ca0.node_internal_area_count = static_cast<u32>(area.size());

		std::vector<u8> emitted_zero;
		const collapse_report cr = collapse(check, m, ca0, &emitted_zero);

		EXPECT_EQ(cr.node_count, base.node_count) << to_string(kind);
		EXPECT_EQ(cr.max_depth, base.max_depth) << to_string(kind);
		ASSERT_EQ(check.nodes().size(), baseline.nodes().size()) << to_string(kind);
		EXPECT_EQ(0, std::memcmp(check.nodes().data(), baseline.nodes().data(),
			check.nodes().size() * sizeof(bvh2_node))) << to_string(kind);
		EXPECT_EQ(check.prim_indices(), baseline.prim_indices()) << to_string(kind);
		ASSERT_EQ(emitted_zero.size(), emitted_base.size()) << to_string(kind);
		EXPECT_EQ(0, std::memcmp(emitted_zero.data(), emitted_base.data(), emitted_zero.size()))
			<< to_string(kind);
	}
}

TEST(GeometryLossAniso, UniformScaleDoesNotChangeWhichDecisionsTheLossChanges)
{
	// The plan predicted the area-based loss would fail a scale test and the
	// ratio-based one would pass. Measurement says otherwise, and the reason is
	// worth recording precisely.
	//
	// Under a uniform scale s the extents go as s, so every face area and every
	// projected triangle area goes as s^2. SA and every loss therefore scale by
	// the same s^2, the DP objective is multiplied by a positive constant, and
	// its argmin is unchanged. Both losses are scale invariant by construction.
	//
	// What is NOT scale invariant is the binary binned-SAH build underneath: it
	// bins on f32 centroids, so a 10x scale perturbs a small number of split
	// decisions. Comparing absolute decision vectors therefore measures the
	// builder, not the loss. The right quantity is the set of decisions the LOSS
	// changes relative to its own scale's SAH baseline.
	mesh small_mesh, big_mesh;
	const std::string p = find_scene("cornell-box.obj");
	ASSERT_FALSE(p.empty());
	ASSERT_TRUE(small_mesh.load_obj(p, 1.0f));
	ASSERT_TRUE(big_mesh.load_obj(p, 10.0f));

	bvh2 small_tree, big_tree;
	build_tree(small_tree, small_mesh);
	build_tree(big_tree, big_mesh);

	ASSERT_EQ(small_tree.nodes().size(), big_tree.nodes().size());

	// Record how much the builder itself moved, so the number below is readable.
	u64 builder_diff = 0;
	for (size_t i = 0; i < small_tree.nodes().size(); ++i)
		if (small_tree.nodes()[i].ptr.raw != big_tree.nodes()[i].ptr.raw) ++builder_diff;

	collapse_args ca;
	ca.width = 8;
	ca.method = collapse_method::dynamic_programming;
	ca.max_leaf_size = 1;
	ca.silent = true;

	auto decisions = [&](const bvh2& src, const mesh& m, collapse_loss kind, double mu)
		{
			geometry_loss_args gla;
			gla.kind = kind;
			gla.mu = mu;
			const std::vector<f32> area = compute_internal_cost_area(src, m, gla);

			bvh2 t = src;
			collapse_args c = ca;
			if (kind != collapse_loss::none && mu != 0.0)
			{
				c.node_internal_area = area.data();
				c.node_internal_area_count = static_cast<u32>(area.size());
			}
			std::vector<u8> emitted;
			collapse(t, m, c, &emitted);
			return emitted;
		};

	const std::vector<u8> base_small = decisions(small_tree, small_mesh, collapse_loss::none, 0.0);
	const std::vector<u8> base_big = decisions(big_tree, big_mesh, collapse_loss::none, 0.0);

	for (collapse_loss kind : { collapse_loss::directional,
								collapse_loss::directional_min })
	{
		const std::vector<u8> loss_small = decisions(small_tree, small_mesh, kind, 1.0);
		const std::vector<u8> loss_big = decisions(big_tree, big_mesh, kind, 1.0);

		ASSERT_EQ(loss_small.size(), loss_big.size());

		// Which decisions did the loss move, at each scale?
		u64 moved_small = 0, moved_big = 0, disagree = 0;
		for (size_t i = 0; i < loss_small.size(); ++i)
		{
			const bool ms = loss_small[i] != base_small[i];
			const bool mb = loss_big[i] != base_big[i];
			if (ms) ++moved_small;
			if (mb) ++moved_big;
			if (ms != mb) ++disagree;
		}

		// The loss must move essentially the same set of decisions at both scales.
		// The tolerance is sized to the builder's own scale sensitivity, since a
		// node the builder moved cannot be expected to match.
		EXPECT_LE(disagree, builder_diff + loss_small.size() / 100)
			<< to_string(kind) << ": loss moved " << moved_small << " decisions at 1x and "
			<< moved_big << " at 10x, disagreeing on " << disagree
			<< "; the binary builder itself differs on " << builder_diff
			<< " of " << small_tree.nodes().size() << " nodes";

		EXPECT_GT(moved_small, 0u) << to_string(kind) << " moved nothing at all";
	}
}

TEST(GeometryLossAniso, MinAndMeanFillAgreeOnIsotropicNodesAndDivergeOtherwise)
{
	const aabb box = equal_face_box();

	// Perfectly isotropic: every axis has the same fill.
	// Tolerances are f32-sized: equal_face_box() is built from sqrt(10.0f), so
	// its face areas are only 10 to f32 precision.
	const directional_geometry iso = g_from(5.0, 5.0, 5.0, 15.0);
	EXPECT_NEAR(directional_min_fill(box, iso), saturating_fill(0.5), 1e-6);
	EXPECT_NEAR(directional_mean_fill(box, iso), 0.5, 1e-6);

	// Anisotropic with the same total: the summed formulation does not move,
	// while the worst-axis one collapses to zero.
	const directional_geometry aniso = g_from(0.0, 7.5, 7.5, 15.0);
	EXPECT_NEAR(directional_mean_fill(box, aniso), 0.5, 1e-6)
		<< "the summed formulation is blind to the redistribution";
	EXPECT_NEAR(directional_min_fill(box, aniso), 0.0, 1e-9);
}
