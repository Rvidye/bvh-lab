#include <build/bvh2_builder.h>
#include <core/traverse_bvh2.h>
#include <eval/directional_geometry.h>
#include <eval/directional_probe.h>
#include <eval/rayset.h>
#include <eval/trace.h>
#include <reference/brute_force.h>
#include <util/camera.h>
#include <util/mesh.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
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

	// Two big diagonal triangles whose AABBs both cover the whole x/y square but
	// which each occupy only one half of it. A ray down -z through (3,1) hits both
	// AABBs, misses the upper-left triangle and hits the lower-right one.
	//
	//   T0 (z = 0): (0,0) (4,4) (0,4)   -- the y >= x half
	//   T1 (z = 1): (0,0) (4,4) (4,0)   -- the y <= x half
	mesh half_square_pair()
	{
		mesh m;
		m.vertices = {
			vec3(0.0f, 0.0f, 0.0f), vec3(4.0f, 4.0f, 0.0f), vec3(0.0f, 4.0f, 0.0f),
			vec3(0.0f, 0.0f, 1.0f), vec3(4.0f, 4.0f, 1.0f), vec3(4.0f, 0.0f, 1.0f),
		};
		m.vertex_indices = { uvec3(0, 1, 2), uvec3(3, 4, 5) };
		finish(m);
		return m;
	}

	// Four coplanar triangles in z == 0. Every node box is flat, so a ray whose
	// direction has u.z == 0 makes every projected box area zero.
	mesh coplanar_strip()
	{
		mesh m;
		for (u32 i = 0; i < 4; ++i)
		{
			const f32 x = f32(i) * 3.0f;
			m.vertices.push_back(vec3(x, 0.0f, 0.0f));
			m.vertices.push_back(vec3(x + 2.0f, 0.0f, 0.0f));
			m.vertices.push_back(vec3(x + 1.0f, 2.0f, 0.0f));
			m.vertex_indices.push_back(uvec3(i * 3 + 0, i * 3 + 1, i * 3 + 2));
		}
		finish(m);
		return m;
	}

	void build(bvh2& tree, mesh& m, split_method method = split_method::binned_sah)
	{
		build_args ba;
		ba.method = method;
		ba.bins = 32;
		ba.max_leaf_size = 1;
		ba.silent = true;

		tree.build(m, ba);
		tree.apply_reorder(m);
		tree.refit(m);
	}

	rayset single_ray(const ray& r)
	{
		rayset rs;
		rs.push(r);
		return rs;
	}

	// Depth-first copy that keeps parents before children and children
	// contiguous, optionally swapping the two children of the root.
	void copy_subtree(const std::vector<bvh2_node>& src, u32 src_id,
		std::vector<bvh2_node>& dst, u32 dst_id, bool swap_children)
	{
		dst[dst_id] = src[src_id];
		if (!src[src_id].ptr.is_int) return;

		const u32 count = src[src_id].ptr.child_cnt;
		const u32 base = static_cast<u32>(dst.size());
		dst[dst_id].ptr.child_idx = base;
		dst.resize(base + count);

		for (u32 c = 0; c < count; ++c)
		{
			const u32 src_child = src[src_id].ptr.child_idx
				+ ((swap_children && count == 2) ? (1u - c) : c);
			copy_subtree(src, src_child, dst, base + c, false);
		}
	}

	std::vector<bvh2_node> mirrored_nodes(const bvh2& tree)
	{
		std::vector<bvh2_node> dst(1);
		copy_subtree(tree.nodes(), 0u, dst, 0u, /*swap_children*/ true);
		return dst;
	}

	std::set<u32> slot_set(const bvh2& tree, u32 node_id)
	{
		std::vector<u32> slots;
		collect_descendant_slots(tree, node_id, slots);
		return std::set<u32>(slots.begin(), slots.end());
	}

	u64 sum(const u64* a, u32 n)
	{
		u64 s = 0;
		for (u32 i = 0; i < n; ++i) s += a[i];
		return s;
	}

} // namespace

// -------------------------------------------------------- labelling basics

TEST(DirectionalProbe, BoxHitWithSubtreeMissIsAnAABBFalsePositive)
{
	mesh m = half_square_pair();
	bvh2 tree;
	build(tree, m);

	const std::vector<directional_geometry> geom = compute_directional_geometry(tree, m);

	const ray r(vec3(3.0f, 1.0f, 5.0f), vec3(0.0f, 0.0f, -1.0f));

	directional_analysis_args args;
	args.query_kind = probe_query_kind::closest_improves_incumbent;
	args.collect_events = true;

	const directional_analysis_result res = analyze_rayset(tree, m, geom, single_ray(r), args);

	// Root pseudo-parent (1 candidate) plus the root node's two children.
	ASSERT_EQ(res.totals.candidate_events, 3u);

	u32 leaf_candidates = 0, positives = 0, negatives = 0;
	for (const candidate_event& e : res.events)
	{
		if (tree.nodes()[e.node_id].ptr.is_int) continue;
		++leaf_candidates;
		if (e.relevant_hit) ++positives; else ++negatives;
	}

	EXPECT_EQ(leaf_candidates, 2u);
	EXPECT_EQ(positives, 1u);
	EXPECT_EQ(negatives, 1u);

	// Exactly one discordant sibling pair at the root node.
	EXPECT_EQ(res.totals.discordant_pairs, 1u);
	EXPECT_EQ(res.totals.rays_with_pairs, 1u);
	ASSERT_EQ(res.pair_rows.size(), 1u);
	EXPECT_EQ(res.pair_rows[0].discordant_pairs, 1u);
	EXPECT_EQ(res.pair_rows[0].candidate_events, 3u);
}

TEST(DirectionalProbe, SiblingPermutationDoesNotChangeChildLabels)
{
	mesh m = half_square_pair();
	bvh2 tree;
	build(tree, m);

	const ray r(vec3(3.0f, 1.0f, 5.0f), vec3(0.0f, 0.0f, -1.0f));

	directional_analysis_args args;
	args.collect_events = true;

	const std::vector<directional_geometry> geom = compute_directional_geometry(tree, m);
	const directional_analysis_result a = analyze_rayset(tree, m, geom, single_ray(r), args);

	// Same tree with the root's two subtrees swapped in storage order.
	bvh2 mirrored = tree;
	mirrored.replace_nodes(mirrored_nodes(tree), 2u);

	const std::vector<directional_geometry> mirrored_geom = compute_directional_geometry(mirrored, m);
	const directional_analysis_result b = analyze_rayset(mirrored, m, mirrored_geom, single_ray(r), args);

	ASSERT_EQ(a.totals.candidate_events, b.totals.candidate_events);
	ASSERT_EQ(a.totals.discordant_pairs, b.totals.discordant_pairs);
	ASSERT_EQ(a.totals.relevant_hit_events, b.totals.relevant_hit_events);

	// Match candidates by their descendant primitive slot set, which is the
	// canonical identity of a subtree, and compare the labels.
	std::map<std::set<u32>, bool> label_a, label_b;
	for (const candidate_event& e : a.events) label_a[slot_set(tree, e.node_id)] = e.relevant_hit;
	for (const candidate_event& e : b.events) label_b[slot_set(mirrored, e.node_id)] = e.relevant_hit;

	EXPECT_EQ(label_a.size(), label_b.size());
	EXPECT_EQ(label_a, label_b);

	// The permutation really did move the hit to the other storage slot.
	const bool first_child_hit_a = a.events[1].relevant_hit;
	const bool first_child_hit_b = b.events[1].relevant_hit;
	EXPECT_NE(first_child_hit_a, first_child_hit_b);
}

TEST(DirectionalProbe, ClosestProbeRejectsTrianglesBeyondTheIncumbent)
{
	mesh m = half_square_pair();
	bvh2 tree;
	build(tree, m);

	// Aim at the lower-right triangle, which sits at z == 1, i.e. t == 4.
	const ray r(vec3(3.0f, 1.0f, 5.0f), vec3(0.0f, 0.0f, -1.0f));

	u32 target = invalid_id;
	for (u32 i = 1; i < tree.nodes().size(); ++i)
	{
		const subtree_probe_result p = probe_subtree_closest(tree, m, i, r, 0.0f, r.t_max);
		if (p.hit) { target = i; break; }
	}
	ASSERT_NE(target, invalid_id);

	const subtree_probe_result generous = probe_subtree_closest(tree, m, target, r, 0.0f, r.t_max);
	ASSERT_TRUE(generous.hit);
	EXPECT_NEAR(generous.t, 4.0f, 1e-4f);

	// An incumbent closer than the triangle must make the child negative.
	const subtree_probe_result tight = probe_subtree_closest(tree, m, target, r, 0.0f, 2.0f);
	EXPECT_FALSE(tight.hit);

	std::vector<u32> slots;
	collect_descendant_slots(tree, target, slots);

	EXPECT_EQ(probe_slots_brute_force(m, tree, slots, r, r.t_max,
		probe_query_kind::closest_improves_incumbent).hit, generous.hit);
	EXPECT_EQ(probe_slots_brute_force(m, tree, slots, r, 2.0f,
		probe_query_kind::closest_improves_incumbent).hit, tight.hit);
}

TEST(DirectionalProbe, FiniteAORangeExcludesAFartherOccluder)
{
	mesh m = half_square_pair();
	bvh2 tree;
	build(tree, m);

	const ray r(vec3(3.0f, 1.0f, 5.0f), vec3(0.0f, 0.0f, -1.0f));

	u32 target = invalid_id;
	for (u32 i = 1; i < tree.nodes().size(); ++i)
	{
		if (probe_subtree_any(tree, m, i, r, r.t_max).hit) { target = i; break; }
	}
	ASSERT_NE(target, invalid_id);

	EXPECT_TRUE(probe_subtree_any(tree, m, target, r, 4.5f).hit);
	EXPECT_FALSE(probe_subtree_any(tree, m, target, r, 3.5f).hit);

	std::vector<u32> slots;
	collect_descendant_slots(tree, target, slots);
	EXPECT_TRUE(probe_slots_brute_force(m, tree, slots, r, 4.5f,
		probe_query_kind::any_occluder_in_range).hit);
	EXPECT_FALSE(probe_slots_brute_force(m, tree, slots, r, 3.5f,
		probe_query_kind::any_occluder_in_range).hit);
}

// -------------------------------------------- production-equality (Gate D0)

namespace {

	struct fixture
	{
		mesh   m;
		bvh2   tree;
		camera cam;
		std::vector<directional_geometry> geom;
	};

	bool load_fixture(fixture& f, const char* scene, u32 res)
	{
		const std::string path = find_scene(scene);
		if (path.empty()) return false;
		if (!f.m.load_obj(path)) return false;

		build(f.tree, f.m);
		f.cam = camera::frame_bounds(f.m.bounds(), res, res);
		f.geom = compute_directional_geometry(f.tree, f.m);
		return true;
	}

} // namespace

TEST(DirectionalProbe, AnalysisTraversalMatchesProductionOnEveryRayDistribution)
{
	// A closed box, so every distribution -- including diffuse_n, which needs a
	// ray to survive several bounces -- produces a non-empty ray set.
	fixture f;
	ASSERT_TRUE(load_fixture(f, "cornell-box.obj", 32));

	rayset_args ra;
	ra.width = 32;
	ra.height = 32;
	ra.incoherent_count = 4096;

	for (ray_distribution dist : all_ray_distributions)
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, ra);
		ASSERT_FALSE(rs.empty()) << to_string(dist);

		const bool any_hit = (dist == ray_distribution::shadow_ao);

		const trace_result production = any_hit
			? occlude_rayset(f.tree, f.m, rs, 1u, 1u)
			: trace_rayset(f.tree, f.m, rs, 1u, 1u);

		for (bool probes : {false, true})
		{
			directional_analysis_args args;
			args.query_kind = query_kind_for(dist);
			args.enable_probes = probes;
			args.collect_ray_hits = true;

			const directional_analysis_result res = analyze_rayset(f.tree, f.m, f.geom, rs, args);

			const char* tag = probes ? "probes on" : "probes off";

			EXPECT_EQ(res.totals.rays, production.rays) << to_string(dist) << " " << tag;
			EXPECT_EQ(res.totals.trace_hits, production.hits) << to_string(dist) << " " << tag;
			EXPECT_EQ(res.totals.trace_node_steps, production.node_steps) << to_string(dist) << " " << tag;
			EXPECT_EQ(res.totals.trace_prim_steps, production.prim_steps) << to_string(dist) << " " << tag;
			EXPECT_EQ(res.totals.trace_box_tests, production.box_tests) << to_string(dist) << " " << tag;
			EXPECT_EQ(res.totals.trace_tri_tests, production.tri_tests) << to_string(dist) << " " << tag;
			EXPECT_EQ(res.totals.trace_max_stack, production.max_stack) << to_string(dist) << " " << tag;

			// Parent visits are internal stack entries, which is exactly what the
			// production kernel counts as a node step.
			EXPECT_EQ(res.totals.parent_visits, production.node_steps) << to_string(dist) << " " << tag;

			// Per-ray equality against the production kernel itself.
			const bvh2_view view = f.tree.view();
			const auto prims = make_prims(f.m, f.tree);

			if (any_hit)
			{
				ASSERT_EQ(res.ray_occluded.size(), size_t(rs.size()));
				for (u32 i = 0; i < rs.size(); ++i)
				{
					null_stats s;
					const bool expected = occluded(view, rs.get(i), prims, s);
					ASSERT_EQ(res.ray_occluded[i] != 0u, expected) << to_string(dist) << " ray " << i;
				}
			}
			else
			{
				ASSERT_EQ(res.ray_hits.size(), size_t(rs.size()));
				std::vector<u32> scratch;
				for (u32 i = 0; i < rs.size(); ++i)
				{
					const ray r = rs.get(i);
					hit expected;
					expected.t = r.t_max;
					null_stats s;
					intersect(view, r, expected, prims, s);

					const hit& got = res.ray_hits[i];
					ASSERT_EQ(got.valid(), expected.valid()) << to_string(dist) << " ray " << i;
					if (!expected.valid()) continue;

					ASSERT_EQ(got.t, expected.t) << to_string(dist) << " ray " << i;
					if (got.id == expected.id) continue;

					// Different ids are legitimate only inside a tie set.
					nearest_tie_set(f.m, r, expected.t, scratch);
					const u32 got_prim = f.tree.prim_index(got.id);
					ASSERT_GT(scratch.size(), 1u) << to_string(dist) << " ray " << i;
					ASSERT_NE(std::find(scratch.begin(), scratch.end(), got_prim), scratch.end())
						<< to_string(dist) << " ray " << i;
				}
			}
		}
	}
}

TEST(DirectionalProbe, ProbesDoNotChangeTheMainTraversal)
{
	fixture f;
	ASSERT_TRUE(load_fixture(f, "cornell-box.obj", 24));

	rayset_args ra;
	ra.width = 24;
	ra.height = 24;
	ra.incoherent_count = 2048;

	for (ray_distribution dist : { ray_distribution::primary, ray_distribution::shadow_ao,
		ray_distribution::diffuse_1, ray_distribution::incoherent })
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, ra);
		ASSERT_FALSE(rs.empty()) << to_string(dist);

		directional_analysis_args off;
		off.query_kind = query_kind_for(dist);
		off.enable_probes = false;
		off.collect_ray_hits = true;

		directional_analysis_args on = off;
		on.enable_probes = true;

		const directional_analysis_result a = analyze_rayset(f.tree, f.m, f.geom, rs, off);
		const directional_analysis_result b = analyze_rayset(f.tree, f.m, f.geom, rs, on);

		EXPECT_EQ(a.totals.trace_node_steps, b.totals.trace_node_steps) << to_string(dist);
		EXPECT_EQ(a.totals.trace_prim_steps, b.totals.trace_prim_steps) << to_string(dist);
		EXPECT_EQ(a.totals.trace_box_tests, b.totals.trace_box_tests) << to_string(dist);
		EXPECT_EQ(a.totals.trace_tri_tests, b.totals.trace_tri_tests) << to_string(dist);
		EXPECT_EQ(a.totals.trace_max_stack, b.totals.trace_max_stack) << to_string(dist);
		EXPECT_EQ(a.totals.trace_hits, b.totals.trace_hits) << to_string(dist);
		EXPECT_EQ(a.totals.parent_visits, b.totals.parent_visits) << to_string(dist);

		EXPECT_EQ(a.ray_hits.size(), b.ray_hits.size()) << to_string(dist);
		for (size_t i = 0; i < a.ray_hits.size(); ++i)
		{
			ASSERT_EQ(a.ray_hits[i].t, b.ray_hits[i].t) << to_string(dist) << " ray " << i;
			ASSERT_EQ(a.ray_hits[i].id, b.ray_hits[i].id) << to_string(dist) << " ray " << i;
		}
		EXPECT_EQ(a.ray_occluded, b.ray_occluded) << to_string(dist);

		// With probes off nothing is labelled, so no events are produced.
		EXPECT_EQ(a.totals.candidate_events, 0u) << to_string(dist);
		EXPECT_GT(b.totals.candidate_events, 0u) << to_string(dist);
	}
}

TEST(DirectionalProbe, ChildLabelsMatchBruteForceOverDescendantSlots)
{
	fixture f;
	ASSERT_TRUE(load_fixture(f, "teapot.obj", 12));

	rayset_args ra;
	ra.width = 12;
	ra.height = 12;
	ra.incoherent_count = 256;

	for (ray_distribution dist : { ray_distribution::primary, ray_distribution::shadow_ao,
		ray_distribution::diffuse_1, ray_distribution::incoherent })
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, ra);
		ASSERT_FALSE(rs.empty()) << to_string(dist);

		directional_analysis_args args;
		args.query_kind = query_kind_for(dist);
		args.collect_events = true;

		const directional_analysis_result res = analyze_rayset(f.tree, f.m, f.geom, rs, args);
		ASSERT_GT(res.events.size(), 0u) << to_string(dist);

		std::vector<u32> slots;
		u64 checked = 0;
		for (const candidate_event& e : res.events)
		{
			collect_descendant_slots(f.tree, e.node_id, slots);

			const subtree_probe_result reference = probe_slots_brute_force(
				f.m, f.tree, slots, rs.get(e.ray_index), e.snapshot_limit, args.query_kind);

			ASSERT_EQ(e.relevant_hit, reference.hit)
				<< to_string(dist) << " ray " << e.ray_index << " node " << e.node_id;
			++checked;
		}
		EXPECT_EQ(checked, res.events.size());
	}
}

// -------------------------------------------------------- reconciliation

TEST(DirectionalProbe, BinAndPairTotalsReconcile)
{
	fixture f;
	ASSERT_TRUE(load_fixture(f, "cornell-box.obj", 24));

	rayset_args ra;
	ra.width = 24;
	ra.height = 24;
	ra.incoherent_count = 2048;

	for (ray_distribution dist : { ray_distribution::primary, ray_distribution::shadow_ao,
		ray_distribution::diffuse_1, ray_distribution::incoherent })
	{
		const rayset rs = rayset::generate(dist, f.m, f.tree, f.cam, ra);
		ASSERT_FALSE(rs.empty()) << to_string(dist);

		directional_analysis_args args;
		args.query_kind = query_kind_for(dist);
		args.collect_events = true;

		const directional_analysis_result res = analyze_rayset(f.tree, f.m, f.geom, rs, args);
		const directional_totals& t = res.totals;
		const bin_accumulator& b = t.bins;

		EXPECT_EQ(t.rays, u64(rs.size())) << to_string(dist);
		EXPECT_EQ(t.parent_visits, t.trace_node_steps) << to_string(dist);

		EXPECT_EQ(sum(b.candidate_count, ratio_bin_count), t.candidate_events) << to_string(dist);
		EXPECT_EQ(sum(b.relevant_hit_count, ratio_bin_count), t.relevant_hit_events) << to_string(dist);
		EXPECT_EQ(sum(b.false_positive_count, ratio_bin_count), t.false_positive_events) << to_string(dist);
		EXPECT_EQ(t.relevant_hit_events + t.false_positive_events, t.candidate_events) << to_string(dist);

		for (u32 i = 0; i < ratio_bin_count; ++i)
			EXPECT_EQ(b.relevant_hit_count[i] + b.false_positive_count[i], b.candidate_count[i])
			<< to_string(dist) << " bin " << to_string(static_cast<ratio_bin>(i));

		EXPECT_EQ(b.candidate_count[u32(ratio_bin::invalid)], t.invalid_events) << to_string(dist);
		EXPECT_EQ(b.candidate_count[u32(ratio_bin::raw_gt_1)], t.saturated_events) << to_string(dist);

		// Candidate events never exceed the box tests that produced them.
		EXPECT_LE(t.candidate_events, t.trace_box_tests) << to_string(dist);

		// Events reconcile with the aggregates.
		EXPECT_EQ(u64(res.events.size()), t.candidate_events) << to_string(dist);

		u64 event_hits = 0, event_invalid = 0, event_saturated = 0;
		u64 event_probe_node_steps = 0;
		for (const candidate_event& e : res.events)
		{
			if (e.relevant_hit) ++event_hits;
			const ratio_bin bin = classify_ratio(e.ratio);
			if (bin == ratio_bin::invalid)  ++event_invalid;
			if (bin == ratio_bin::raw_gt_1) ++event_saturated;
			event_probe_node_steps += e.probe.node_steps;
		}
		EXPECT_EQ(event_hits, t.relevant_hit_events) << to_string(dist);
		EXPECT_EQ(event_invalid, t.invalid_events) << to_string(dist);
		EXPECT_EQ(event_saturated, t.saturated_events) << to_string(dist);
		EXPECT_EQ(event_probe_node_steps, sum(b.probe_node_steps, ratio_bin_count)) << to_string(dist);

		// Pair rows reconcile with the aggregates.
		EXPECT_EQ(u64(res.pair_rows.size()), t.rays_with_pairs) << to_string(dist);

		u64 row_pairs = 0, row_candidates = 0;
		u64 row_correct[score_count]{}, row_ties[score_count]{};
		for (const ray_pair_record& row : res.pair_rows)
		{
			EXPECT_GT(row.discordant_pairs, 0u) << to_string(dist);
			row_pairs += row.discordant_pairs;
			row_candidates += row.candidate_events;
			for (u32 s = 0; s < score_count; ++s)
			{
				row_correct[s] += row.correct[s];
				row_ties[s] += row.ties[s];
			}
		}
		EXPECT_EQ(row_pairs, t.discordant_pairs) << to_string(dist);
		EXPECT_EQ(row_candidates, t.candidate_events_in_pair_rows) << to_string(dist);
		for (u32 s = 0; s < score_count; ++s)
		{
			EXPECT_EQ(row_correct[s], t.correct[s]) << to_string(dist) << " " << to_string(score_id(s));
			EXPECT_EQ(row_ties[s], t.ties[s]) << to_string(dist) << " " << to_string(score_id(s));
			EXPECT_LE(t.correct[s] + t.ties[s], t.discordant_pairs)
				<< to_string(dist) << " " << to_string(score_id(s));
		}

		// A binary BVH gives at most one discordant pair per parent visit.
		EXPECT_LE(t.discordant_pairs, t.parent_visits) << to_string(dist);
	}
}

TEST(DirectionalProbe, InvalidDescriptorsOnlyEnterTheInvalidBucket)
{
	mesh m = coplanar_strip();
	bvh2 tree;
	build(tree, m);

	const std::vector<directional_geometry> geom = compute_directional_geometry(tree, m);

	// Every node box is flat in z; a ray travelling along +x inside that plane
	// has u.z == 0, so every projected box area is exactly zero.
	rayset rs;
	for (u32 i = 0; i < 4; ++i)
		rs.push(ray(vec3(-10.0f, 0.5f + f32(i) * 0.25f, 0.0f), vec3(1.0f, 0.0f, 0.0f)));

	directional_analysis_args args;
	args.collect_events = true;

	const directional_analysis_result res = analyze_rayset(tree, m, geom, rs, args);

	ASSERT_GT(res.totals.candidate_events, 0u);
	EXPECT_EQ(res.totals.invalid_events, res.totals.candidate_events);
	EXPECT_EQ(res.totals.bins.candidate_count[u32(ratio_bin::invalid)], res.totals.candidate_events);

	for (u32 i = 0; i < ratio_bin_count; ++i)
	{
		if (i == u32(ratio_bin::invalid)) continue;
		EXPECT_EQ(res.totals.bins.candidate_count[i], 0u) << to_string(static_cast<ratio_bin>(i));
	}

	for (const candidate_event& e : res.events)
	{
		EXPECT_FALSE(e.ratio.valid);
		EXPECT_EQ(classify_ratio(e.ratio), ratio_bin::invalid);
		EXPECT_FALSE(e.scores.defined[u32(score_id::directional)]);
	}

	// Any discordant pair here must be a tie for the directional score, never a
	// correct call, because the score is undefined on both sides.
	EXPECT_EQ(res.totals.correct[u32(score_id::directional)], 0u);
	EXPECT_EQ(res.totals.ties[u32(score_id::directional)], res.totals.discordant_pairs);
}

TEST(DirectionalProbe, ValidEventsNeverLandInTheInvalidBucket)
{
	fixture f;
	ASSERT_TRUE(load_fixture(f, "bunny.obj", 16));

	rayset_args ra;
	ra.width = 16;
	ra.height = 16;
	ra.incoherent_count = 512;

	const rayset rs = rayset::generate(ray_distribution::incoherent, f.m, f.tree, f.cam, ra);
	ASSERT_FALSE(rs.empty());

	directional_analysis_args args;
	args.collect_events = true;

	const directional_analysis_result res = analyze_rayset(f.tree, f.m, f.geom, rs, args);
	ASSERT_GT(res.events.size(), 0u);

	u64 valid = 0;
	for (const candidate_event& e : res.events)
	{
		const ratio_bin bin = classify_ratio(e.ratio);
		if (e.ratio.valid)
		{
			++valid;
			EXPECT_NE(bin, ratio_bin::invalid);
			EXPECT_TRUE(std::isfinite(e.ratio.raw));
			EXPECT_GE(e.ratio.raw, 0.0);
		}
		else
		{
			EXPECT_EQ(bin, ratio_bin::invalid);
		}
	}
	EXPECT_GT(valid, 0u);
	EXPECT_EQ(res.totals.candidate_events - res.totals.invalid_events, valid);
}

TEST(DirectionalProbe, ProbeCountersMatchAFreshIndependentQuery)
{
	fixture f;
	ASSERT_TRUE(load_fixture(f, "teapot.obj", 10));

	rayset_args ra;
	ra.width = 10;
	ra.height = 10;

	const rayset rs = rayset::generate(ray_distribution::primary, f.m, f.tree, f.cam, ra);
	ASSERT_FALSE(rs.empty());

	directional_analysis_args args;
	args.collect_events = true;

	const directional_analysis_result res = analyze_rayset(f.tree, f.m, f.geom, rs, args);
	ASSERT_GT(res.events.size(), 0u);

	for (const candidate_event& e : res.events)
	{
		const subtree_probe_result p = probe_subtree_closest(f.tree, f.m, e.node_id,
			rs.get(e.ray_index), e.box_t, e.snapshot_limit);

		ASSERT_EQ(p.hit, e.relevant_hit) << "node " << e.node_id;
		ASSERT_EQ(p.stats.node_steps, e.probe.node_steps) << "node " << e.node_id;
		ASSERT_EQ(p.stats.prim_steps, e.probe.prim_steps) << "node " << e.node_id;
		ASSERT_EQ(p.stats.box_tests, e.probe.box_tests) << "node " << e.node_id;
		ASSERT_EQ(p.stats.tri_tests, e.probe.tri_tests) << "node " << e.node_id;
	}
}

TEST(DirectionalProbe, AnalysisIsDeterministic)
{
	fixture f;
	ASSERT_TRUE(load_fixture(f, "teapot.obj", 16));

	rayset_args ra;
	ra.width = 16;
	ra.height = 16;

	const rayset rs = rayset::generate(ray_distribution::diffuse_1, f.m, f.tree, f.cam, ra);
	ASSERT_FALSE(rs.empty());

	directional_analysis_args args;
	const directional_analysis_result a = analyze_rayset(f.tree, f.m, f.geom, rs, args);
	const directional_analysis_result b = analyze_rayset(f.tree, f.m, f.geom, rs, args);

	EXPECT_EQ(a.totals.candidate_events, b.totals.candidate_events);
	EXPECT_EQ(a.totals.discordant_pairs, b.totals.discordant_pairs);
	EXPECT_EQ(a.pair_rows.size(), b.pair_rows.size());

	for (u32 s = 0; s < score_count; ++s)
	{
		EXPECT_EQ(a.totals.correct[s], b.totals.correct[s]) << to_string(score_id(s));
		EXPECT_EQ(a.totals.ties[s], b.totals.ties[s]) << to_string(score_id(s));
	}
	for (u32 i = 0; i < ratio_bin_count; ++i)
	{
		EXPECT_EQ(a.totals.bins.candidate_count[i], b.totals.bins.candidate_count[i]);
		EXPECT_EQ(a.totals.bins.probe_node_steps[i], b.totals.bins.probe_node_steps[i]);
	}
	for (size_t i = 0; i < a.pair_rows.size(); ++i)
	{
		ASSERT_EQ(a.pair_rows[i].ray_index, b.pair_rows[i].ray_index);
		ASSERT_EQ(a.pair_rows[i].discordant_pairs, b.pair_rows[i].discordant_pairs);
	}
}

TEST(DirectionalProbe, DescendantSlotsCoverEveryPrimitiveExactlyOnce)
{
	mesh m;
	const std::string path = find_scene("teapot.obj");
	ASSERT_FALSE(path.empty());
	ASSERT_TRUE(m.load_obj(path));

	bvh2 tree;
	build(tree, m);

	std::vector<u32> slots;
	collect_descendant_slots(tree, 0u, slots);

	ASSERT_EQ(slots.size(), size_t(m.triangle_count()));
	for (u32 i = 0; i < slots.size(); ++i) EXPECT_EQ(slots[i], i);

	// A child's slot set is a subset of its parent's, and siblings are disjoint.
	for (u32 i = 0; i < tree.nodes().size(); ++i)
	{
		const bvh2_node& node = tree.nodes()[i];
		if (!node.ptr.is_int) continue;

		const std::set<u32> parent = slot_set(tree, i);
		std::set<u32> union_of_children;
		for (u32 c = 0; c < node.ptr.child_cnt; ++c)
		{
			const std::set<u32> child = slot_set(tree, node.ptr.child_idx + c);
			for (u32 s : child)
			{
				ASSERT_TRUE(parent.count(s) != 0);
				ASSERT_TRUE(union_of_children.insert(s).second);
			}
		}
		ASSERT_EQ(union_of_children, parent);
	}
}
