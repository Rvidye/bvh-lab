#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <core/bvh2.h>
#include <core/ray.h>
#include <core/trace_stats.h>
#include <eval/directional_geometry.h>
#include <eval/rayset.h>
#include <util/mesh.h>

#include <vector>

namespace bvh
{
	// Direction D: unbiased, host-only child probes.
	//
	// Normal traversal censors the labels we need. Closest-hit tightens h.t and
	// prunes later stack entries; any-hit stops at its first primitive. Whether a
	// particular child subtree contains a relevant intersection would then depend
	// on storage order and on earlier hits.
	//
	// So: mirror the production traversal in host code, and at every parent visit
	// snapshot the active range limit, box-test the children exactly as the
	// production kernel does, and run an INDEPENDENT query into each box-hit
	// child using that same snapshot. A probe never touches the analysis
	// traversal's hit record, stack or counters.
	//
	// Probe counters are counterfactual child costs. They are labelled probe_*
	// and they are not the work the production traversal actually performed.

	enum class probe_query_kind : u32
	{
		// primary / reflection / diffuse_* / incoherent
		closest_improves_incumbent,
		// shadow_ao: finite ambient occlusion, NOT a direct-light shadow workload
		any_occluder_in_range,
	};

	const char* to_string(probe_query_kind k);
	probe_query_kind query_kind_for(ray_distribution d);

	// The five scores compared on every candidate event. Frozen in
	// experiments/direction_d/README.md section 5. Higher predicts "this child
	// contains the relevant intersection" for all five.
	enum class score_id : u32
	{
		directional = 0,          // per-ray-direction fill ratio, geom_proj/box_proj along d
		surface_density,
		primitive_count,
		box_surface_ratio,
		box_projected_ratio,      // pure geometry: uses no triangle information at all
		// The two collapse-loss shapes, scored as FILLS so that "higher predicts
		// the child containing the hit" holds for them as for every other score.
		// mean_fill is 1 - Ldir/SA, i.e. the summed formulation the collapse
		// currently optimises; min_fill is the worst-axis formulation. Comparing
		// these two is the whole of Gate B.
		directional_mean_fill,
		directional_min_fill,
		count
	};

	constexpr u32 score_count = static_cast<u32>(score_id::count);

	const char* to_string(score_id s);

	struct candidate_scores
	{
		double value[score_count]{};
		bool   defined[score_count]{};
	};

	// One independent subtree query.
	struct subtree_probe_result
	{
		bool        hit{ false };
		f32         t{ 0.0f };
		u32         prim_slot{ invalid_id };
		trace_stats stats;
	};

	// One box-hit child at one parent visit. Only collected when
	// directional_analysis_args::collect_events is set; the full runs keep only
	// the aggregates below.
	struct candidate_event
	{
		u32  ray_index{ 0 };
		u32  parent_visit{ 0 };   // ordinal of the parent visit within this ray
		u32  parent_node_id{ invalid_id };
		u32  node_id{ 0 };
		f32  box_t{ 0.0f };
		f32  snapshot_limit{ 0.0f };
		bool relevant_hit{ false };
		directional_ratio ratio{};
		candidate_scores  scores{};
		trace_stats       probe{};
	};

	// Aggregates over the frozen ratio bins.
	struct bin_accumulator
	{
		u64 candidate_count[ratio_bin_count]{};
		u64 relevant_hit_count[ratio_bin_count]{};
		u64 false_positive_count[ratio_bin_count]{};

		u64 probe_node_steps[ratio_bin_count]{};
		u64 probe_prim_steps[ratio_bin_count]{};
		u64 probe_box_tests[ratio_bin_count]{};
		u64 probe_tri_tests[ratio_bin_count]{};

		u64 false_positive_node_steps[ratio_bin_count]{};
		u64 false_positive_prim_steps[ratio_bin_count]{};
		u64 false_positive_box_tests[ratio_bin_count]{};
		u64 false_positive_tri_tests[ratio_bin_count]{};
	};

	// One row of d1_directional_pairs.csv. Written only for rays that produced at
	// least one discordant sibling pair; the ray is the resampling cluster.
	struct ray_pair_record
	{
		u32 ray_index{ 0 };
		u32 candidate_events{ 0 };
		u32 discordant_pairs{ 0 };
		u32 correct[score_count]{};
		u32 ties[score_count]{};
	};

	struct directional_totals
	{
		u64 rays{ 0 };
		u64 rays_with_pairs{ 0 };
		u64 parent_visits{ 0 };
		u64 candidate_events{ 0 };
		u64 candidate_events_in_pair_rows{ 0 };
		u64 relevant_hit_events{ 0 };
		u64 false_positive_events{ 0 };
		u64 invalid_events{ 0 };
		u64 saturated_events{ 0 };

		u64 discordant_pairs{ 0 };
		u64 correct[score_count]{};
		u64 ties[score_count]{};

		// Ordinary production traversal counters, kept on their own scale.
		u64 trace_node_steps{ 0 };
		u64 trace_prim_steps{ 0 };
		u64 trace_box_tests{ 0 };
		u64 trace_tri_tests{ 0 };
		u32 trace_max_stack{ 0 };
		u64 trace_hits{ 0 };

		bin_accumulator bins{};
	};

	struct directional_analysis_args
	{
		probe_query_kind query_kind{ probe_query_kind::closest_improves_incumbent };
		bool enable_probes{ true };     // false reproduces production exactly
		bool collect_pair_rows{ true };
		bool collect_events{ false };    // per-candidate records; tests and fixtures only
		bool collect_ray_hits{ false };  // per-ray results, for the production-equality gate
	};

	struct directional_analysis_result
	{
		directional_totals           totals{};
		std::vector<ray_pair_record> pair_rows;
		std::vector<candidate_event> events;

		// Only filled when collect_ray_hits is set. ray_hits is the closest-hit
		// record; ray_occluded is the any-hit answer.
		std::vector<hit> ray_hits;
		std::vector<u8>  ray_occluded;
	};

	directional_analysis_result analyze_rayset(const bvh2& tree, const mesh& m,
		const std::vector<directional_geometry>& geom,
		const rayset& rays,
		const directional_analysis_args& args = {});

	// ------------------------------------------------------------------ probes

	// An independent query starting at an already-box-tested child. The child's
	// own AABB test is NOT re-counted: the probe covers the work below the child.
	subtree_probe_result probe_subtree_closest(const bvh2& tree, const mesh& m,
		u32 child_node_id, const ray& r, f32 entry_t, f32 limit);

	subtree_probe_result probe_subtree_any(const bvh2& tree, const mesh& m,
		u32 child_node_id, const ray& r, f32 limit);

	// Every primitive slot below a node, in slot order.
	void collect_descendant_slots(const bvh2& tree, u32 node_id, std::vector<u32>& out);

	// Brute-force reference for a probe: a linear scan over the child's own
	// descendant primitive slots, with the same limit and the same primitive
	// intersector the probe uses.
	subtree_probe_result probe_slots_brute_force(const mesh& m, const bvh2& tree,
		const std::vector<u32>& slots, const ray& r, f32 limit, probe_query_kind kind);

} // namespace bvh
