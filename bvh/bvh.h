#pragma once

#include <core/qualifiers.h>

namespace bvh 
{
	//core types
	struct vec2;
	struct vec3;
	struct vec4;
	struct uvec3;
	struct aabb;
	struct ray;
	struct hit;
	struct triagnle;
	struct rng;
	struct null_stats;
	struct trace_stats;

	//utils types
	class mesh;
	class camera;
	class image;
	class matrics;

	// core/ layouts
	union  bvh_ptr;
	struct bvh2_node;
	struct bvh2_view;

	// build types /bvh's
	class bvh2;
	enum class split_method;
	struct build_args;
	struct build_report;

	// eval
	struct quality_metrics;
	struct quality_args;
	struct trace_result;
	struct directional_geometry;
	struct directional_ratio;
	struct unit_direction;
	struct candidate_event;
	struct directional_totals;
	struct directional_analysis_result;

} // namespace bvh


