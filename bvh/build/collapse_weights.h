#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <eval/direction_stats.h>

#include <vector>

namespace bvh
{
	// Per-node collapse weight, replacing aabb::surface_area() in the DP.
	//
	//   Aeff(n) = (1 - lambda) * SA(B_n) + lambda * 4 * Adir(B_n; W)
	//
	// The factor 4 puts both terms in the same units: E_isotropic[Aproj] = SA/4.
	//
	// At lambda == 0 the weights are produced by calling aabb::surface_area()
	// itself, so the DP consumes numerically identical inputs through identical
	// arithmetic and the emitted tree is byte-identical to the ordinary collapse.
	// That is a structural guarantee, not a numerical hope: computing
	// 2*(Fx+Fy+Fz) instead could differ by an ULP and flip a '<' in the DP.
	std::vector<f32> compute_collapse_weights(const bvh2& tree, double lambda,
		const direction_weights& directions);

} // namespace bvh
