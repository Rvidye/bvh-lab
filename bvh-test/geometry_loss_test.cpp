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
