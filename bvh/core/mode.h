#pragma once

#include <core/qualifiers.h>

namespace bvh
{
	constexpr f32 degenerate_det_epsilon = 1e-12f;

	struct robust_mode
	{
		static constexpr bool slab_padding = true; // Ize 2013: fixes ray missing far bounds
		static constexpr bool axis_parallel_guard = true; // explicit handling for a direction compenent of exactly zero
		static constexpr bool reject_degenerate = true; // NaN can never reacch a hit record
		static constexpr const char* name = "robust";
	};

	struct arches_mode
	{
		static constexpr bool slab_padding = false;
		static constexpr bool axis_parallel_guard = false;
		static constexpr bool reject_degenerate = false;
		static constexpr const char* name = "arches";
	};

	using default_mode = robust_mode;
} // namespace bvh
