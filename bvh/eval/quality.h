#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <util/mesh.h>

namespace bvh {

	// Tree quality metrics.
	struct quality_metrics
	{
		double sah_cost{ 0.0 };
		double sah_cost_arches{ 0.0 };
		double sah_cost_slots{ 0.0 };
		double epo{ 0.0 };
		double combined{ 0.0 };

		u32    node_count{ 0 };
		u32    leaf_count{ 0 };
		u32    interior_count{ 0 };
		u32    max_depth{ 0 };
		double mean_leaf_size{ 0.0 };
		size_t bytes{ 0 };
		double bytes_per_tri{ 0.0 };
	};

	struct quality_args
	{
		double c_traversal{ 1.0 };
		double c_intersect{ 1.0 };
		bool compute_epo{ false };
		double epo_weight{ 0.71 };
	};

	quality_metrics evaluate(const bvh2& tree, const mesh& m, const quality_args& args = {});

} // namespace bvh
