#pragma once

#include <bvh.h>
#include <eval/rayset.h>

namespace bvh
{
	// Ratio-of-means direction moments over a training ray set.
	//
	// Aproj(B,d) = |d.x|*Fx + |d.y|*Fy + |d.z|*Fz is linear in (|d.x|,|d.y|,|d.z|),
	// so the sum over training directions of Aproj collapses exactly to
	//   Adir(B) = wx*Fx + wy*Fy + wz*Fz,   w = (E|d.x|, E|d.y|, E|d.z|).
	// Isotropic directions give w = (1/2,1/2,1/2) and 4*Adir(B) == SA(B).
	struct direction_weights
	{
		double wx{ 0.5 }, wy{ 0.5 }, wz{ 0.5 };
		u64    rays{ 0 };
		u64    rejected{ 0 };
		bool   valid{ false };
	};

	// Accumulated in double, in ray-index order, so the result is deterministic.
	// Directions are normalised per ray; rayset directions are not normalised at
	// generation.
	direction_weights compute_direction_weights(const rayset& training);

} // namespace bvh
