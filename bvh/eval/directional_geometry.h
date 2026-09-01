#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <core/aabb.h>
#include <core/ray.h>
#include <core/vec.h>
#include <util/mesh.h>

#include <vector>

namespace bvh
{
	// Direction D: the projected-geometry descriptor.
	//
	// This is a SURFACE RATIO and an EMPTINESS LOWER BOUND, not a probability.
	// It knows nothing about ray origins, finite ranges, visibility, occlusion,
	// or how much of the projected geometry overlaps itself.
	//
	// The descriptor lives in a side vector indexed by node id. bvh2_node is
	// deliberately left alone.

	// Sum over a node's descendant triangles of the axis-plane projected areas.
	// p_yz == G.x, p_xz == G.y, p_xy == G.z.
	struct directional_geometry
	{
		double p_yz{ 0.0 };
		double p_xz{ 0.0 };
		double p_xy{ 0.0 };
		double triangle_surface_area_sum{ 0.0 }; // analysis control, not part of G
		u32    primitive_count{ 0 };

		void add(const directional_geometry& o)
		{
			p_yz += o.p_yz;
			p_xz += o.p_xz;
			p_xy += o.p_xy;
			triangle_surface_area_sum += o.triangle_surface_area_sum;
			primitive_count += o.primitive_count;
		}
	};

	// Q_raw / Q_clamped / E_lower for one node and one direction.
	// When valid == false, raw/clamped/emptiness_lower are NaN: an invalid
	// descriptor is never silently mapped to 0 or 1.
	struct directional_ratio
	{
		double raw{ 0.0 };
		double clamped{ 0.0 };
		double emptiness_lower{ 0.0 };
		bool   valid{ false };
	};

	// A finite, unit-length direction in double precision. valid == false when
	// the source direction was zero or non-finite.
	struct unit_direction
	{
		double x{ 0.0 };
		double y{ 0.0 };
		double z{ 0.0 };
		bool   valid{ false };
	};

	unit_direction normalize_direction(const vec3& d);

	directional_geometry triangle_directional_geometry(const triangle& t);

	// P_exact for one triangle: 0.5 * abs(dot(c, u)). Returns NaN for an
	// invalid direction.
	double triangle_exact_projected_area(const triangle& t, const unit_direction& u);
	double triangle_exact_projected_area(const triangle& t, const vec3& direction);

	// Node-indexed side array. Parents precede children in every tree this
	// repository builds, so a reverse node scan is a valid accumulation order.
	// Leaves read their primitive slots through tree.prim_index(), so the result
	// is identical before and after apply_reorder(). Works for any node width.
	std::vector<directional_geometry> compute_directional_geometry(const bvh2& tree, const mesh& m);

	// U_axis. Returns NaN for an invalid direction.
	double axis_projected_area_upper(const directional_geometry& g, const unit_direction& u);
	double axis_projected_area_upper(const directional_geometry& g, const vec3& direction);

	// B_box. Returns NaN for an invalid direction; returns 0 for a box with no
	// projected extent, which the ratio then reports as invalid.
	double projected_box_area(const aabb& box, const unit_direction& u);
	double projected_box_area(const aabb& box, const vec3& direction);

	directional_ratio compute_directional_ratio(const directional_geometry& g, const aabb& box,
		const unit_direction& u);
	directional_ratio compute_directional_ratio(const directional_geometry& g, const aabb& box,
		const vec3& direction);

	// Ratio bins, frozen in experiments/direction_d/README.md section 4.
	enum class ratio_bin : u32
	{
		q_00_10 = 0, q_10_20, q_20_30, q_30_40, q_40_50,
		q_50_60, q_60_70, q_70_80, q_80_90, q_90_100,
		raw_gt_1,
		invalid,
		count
	};

	constexpr u32 ratio_bin_count = static_cast<u32>(ratio_bin::count);

	const char* to_string(ratio_bin b);

	// An invalid ratio can only ever land in ratio_bin::invalid.
	ratio_bin classify_ratio(const directional_ratio& r);

} // namespace bvh
