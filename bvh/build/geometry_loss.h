#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <core/aabb.h>
#include <eval/directional_geometry.h>
#include <util/mesh.h>

#include <vector>

namespace bvh
{
	// Geometry-derived loss terms for binary-to-wide collapse.
	//
	// NOTHING here depends on rays, cameras, views or workloads. Every quantity
	// is accumulated from the triangles inside each binary subtree, using the
	// descriptor already implemented and tested in eval/directional_geometry.
	//
	// For a node AABB with extents (ex, ey, ez):
	//
	//     Fx = ey*ez        Fy = ex*ez        Fz = ex*ey
	//     SA = 2 * (Fx + Fy + Fz)
	//
	// With G(n) the bottom-up sum of per-triangle axis projections
	// (0.5*|c.x|, 0.5*|c.y|, 0.5*|c.z|) and Atri(n) the bottom-up sum of
	// triangle surface areas:
	//
	//     Ex = max(Fx - G.x, 0)    Ey = max(Fy - G.y, 0)    Ez = max(Fz - G.z, 0)
	//     Ldir(n)    = 2 * (Ex + Ey + Ez)
	//     Lscalar(n) = max(SA(n) - 2 * Atri(n), 0)
	//
	// Ldir is the surface-area-scaled average lower bound on projected empty area
	// along the six signed coordinate-axis directions. It is deliberately
	// one-sided: projected triangle areas overlap, so an axis where G exceeds the
	// box projection contributes zero loss rather than a negative one.
	//
	// Lscalar is the matched non-directional control. It has the same units and
	// the same [0, SA] range but uses only total triangle area, so comparing the
	// two separates a directional-component effect from a triangle-density one.

	enum class collapse_loss : u32
	{
		none,             // ordinary SAH collapse
		directional,      // SAH + mu * Ldir
		scalar_density,   // SAH + mu * Lscalar  (control)
	};

	const char* to_string(collapse_loss kind);

	struct geometry_loss_args
	{
		collapse_loss kind{ collapse_loss::none };
		double        mu{ 0.0 };
	};

	// Per-node loss values, one entry per node of the supplied tree.
	struct geometry_loss_terms
	{
		std::vector<f32> ldir;       // Ldir(n)
		std::vector<f32> lscalar;    // Lscalar(n)
		std::vector<f32> sa;         // SA(n), from aabb::surface_area()
	};

	geometry_loss_terms compute_geometry_loss(const bvh2& tree, const mesh& m);

	// Loss evaluated for a single node, exposed for tests.
	f32 directional_loss(const aabb& box, const directional_geometry& g);
	f32 scalar_density_loss(const aabb& box, const directional_geometry& g);

	// The array handed to collapse_args::node_internal_area:
	//     SA(n) + mu * L(n)
	// At mu == 0, or with collapse_loss::none, every entry is produced by calling
	// aabb::surface_area() itself, so the dynamic program consumes numerically
	// identical inputs and the emitted tree is byte-identical to the ordinary
	// collapse.
	std::vector<f32> compute_internal_cost_area(const bvh2& tree, const mesh& m,
		const geometry_loss_args& args);

	// Sum of L(n) over the INTERNAL nodes of the given tree, divided by the root
	// surface area, so it is on the same normalised scale as quality::sah_cost.
	struct loss_totals
	{
		double directional{ 0.0 };
		double scalar_density{ 0.0 };
		u64    internal_nodes{ 0 };
	};

	loss_totals sum_internal_loss(const bvh2& tree, const mesh& m);

} // namespace bvh
