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

	// WHY THE SUM IS THE PROBLEM
	//
	// Ldir adds the per-axis empty AREAS. Two nodes with opposite geometric
	// character therefore receive the same loss:
	//
	//   A: per-axis empty [10, 0, 0]   Ldir = 20   per-axis fill [0.00, 1.00, 1.00]
	//   B: per-axis empty [10/3 x3]    Ldir = 20   per-axis fill [0.67, 0.67, 0.67]
	//
	// A has an axis along which it is entirely empty, which is the pathology the
	// idea exists to find. B is merely a uniformly loose box, which SAH already
	// penalises. Summing over the axes is the isotropic case, so it erases
	// exactly the directional signal it was meant to capture.
	//
	// The variants below keep the per-axis structure by taking the WORST axis
	// rather than the sum, and work in fill RATIOS rather than absolute areas so
	// the loss is not dominated by whichever face happens to be largest.

	enum class collapse_loss : u32
	{
		none,                 // ordinary SAH collapse
		directional,          // SAH + mu * Ldir      -- summed areas, kept as control
		scalar_density,       // SAH + mu * Lscalar   -- density control
		directional_min,      // SAH + mu * SA*(1 - min fill)
		directional_softmin,  // SAH + mu * SA*(1 - softmin(fill, beta))
		directional_spread,   // SAH + mu * SA*(max fill - min fill)
	};

	const char* to_string(collapse_loss kind);

	// A degenerate axis has zero projected box face, so its fill ratio is 0/0.
	// min() is far more sensitive to this than sum() was, and a careless guard
	// would silently turn the loss into a heuristic, so the choice is explicit,
	// recorded and tested both ways.
	enum class degenerate_axis_policy : u32
	{
		// "No extent" and "full" are different claims; do not conflate them.
		exclude,        // drop the axis from min/max/spread; none valid -> loss 0
		treat_as_full,  // fill = 1: nothing to enter, so no waste
	};

	// beta = 0 is the fill-weighted mean, beta -> inf is the min.
	constexpr double default_softmin_beta = 4.0;

	struct geometry_loss_args
	{
		collapse_loss          kind{ collapse_loss::none };
		double                 mu{ 0.0 };
		degenerate_axis_policy degenerate{ degenerate_axis_policy::exclude };
		double                 softmin_beta{ default_softmin_beta };
	};

	// Per-axis fill ratios for one node. q is the raw unclamped ratio
	// G_i / F_i; fill is the saturating map below. Exposed for the node-term
	// dump so the saturation regime can be inspected rather than guessed at.
	struct axis_fill
	{
		double q[3]{ 0.0, 0.0, 0.0 };
		double fill[3]{ 0.0, 0.0, 0.0 };
		bool   valid[3]{ false, false, false };
		bool   saturated[3]{ false, false, false };   // q >= 1
		u32    valid_count{ 0 };
	};

	// Projected triangle areas overlap, so q exceeds 1 in dense geometry.
	// Clamping maps q = 2 and q = 10 to the same value, and in a scene where most
	// nodes saturate the loss goes nearly constant. 1 - exp(-q) is the correct
	// estimator if triangles are modelled as independent occluders, maps
	// [0, inf) -> [0, 1) smoothly, and keeps the ordering where clamping is flat.
	//   q=0 -> 0, q=1 -> 0.632, q=3 -> 0.950, q=10 -> 0.99995
	double saturating_fill(double q);

	axis_fill compute_axis_fill(const aabb& box, const directional_geometry& g);

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

	f32 directional_min_loss(const aabb& box, const directional_geometry& g,
		degenerate_axis_policy policy = degenerate_axis_policy::exclude);
	f32 directional_softmin_loss(const aabb& box, const directional_geometry& g,
		double beta = default_softmin_beta,
		degenerate_axis_policy policy = degenerate_axis_policy::exclude);
	f32 directional_spread_loss(const aabb& box, const directional_geometry& g,
		degenerate_axis_policy policy = degenerate_axis_policy::exclude);

	// Fill shapes in [0,1], higher meaning fuller. These are what the sibling
	// probe scores, so "higher predicts the child that contains the hit" holds
	// for them exactly as it does for the existing scores.
	//   mean_fill is 1 - Ldir/SA, i.e. the CURRENT summed formulation expressed
	//   as a fill, so min_fill and mean_fill are directly comparable.
	double directional_mean_fill(const aabb& box, const directional_geometry& g);
	double directional_min_fill(const aabb& box, const directional_geometry& g,
		degenerate_axis_policy policy = degenerate_axis_policy::exclude);

	// The array handed to collapse_args::node_internal_area:
	//     SA(n) + mu * L(n)
	// At mu == 0, or with collapse_loss::none, every entry is produced by calling
	// aabb::surface_area() itself, so the dynamic program consumes numerically
	// identical inputs and the emitted tree is byte-identical to the ordinary
	// collapse.
	std::vector<f32> compute_internal_cost_area(const bvh2& tree, const mesh& m,
		const geometry_loss_args& args);

	// Loss for one node under any variant, so callers do not re-switch.
	f32 evaluate_loss(const aabb& box, const directional_geometry& g,
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
