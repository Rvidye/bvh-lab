#include <eval/directional_probe.h>

#include <build/geometry_loss.h>
#include <core/isect.h>
#include <core/mode.h>
#include <core/traverse_bvh2.h>
#include <eval/trace.h>
#include <util/check.h>

#include <cmath>
#include <limits>

namespace bvh
{
	namespace
	{
		// The production kernels this analysis mirrors run in robust mode.
		using analysis_mode = robust_mode;

		// bvh_ptr::child_cnt is five bits.
		constexpr u32 max_children = 31u;

		// A probe stack that grows would mean a tree deeper than the production
		// traversal can handle anyway; keep a hard ceiling so a runaway is loud.
		constexpr size_t probe_stack_limit = 4096;

		struct probe_entry
		{
			f32     t;
			bvh_ptr ptr;
		};

		// Scratch reused across every ray of one analysis run.
		struct probe_scratch
		{
			std::vector<probe_entry> closest_stack;
			std::vector<bvh_ptr>     any_stack;
		};

		subtree_probe_result probe_closest(const bvh2_view& view, const mesh_prims<analysis_mode>& prim,
			u32 child_node_id, const ray& r, f32 entry_t, f32 limit,
			std::vector<probe_entry>& stack)
		{
			subtree_probe_result out;

			const vec3 inv_d = rcp(r.d);

			hit h;
			h.t = limit;

			stack.clear();
			stack.push_back(probe_entry{ entry_t, view.nodes[child_node_id].ptr });

			while (!stack.empty())
			{
				const probe_entry entry = stack.back();
				stack.pop_back();

				if (entry.t >= h.t) continue;

				if (entry.ptr.is_int)
				{
					out.stats.node_step();

					const size_t insert_base = stack.size();

					for (u32 i = 0; i < entry.ptr.child_cnt; ++i)
					{
						const u32 node_id = entry.ptr.child_idx + i;
						out.stats.box_test();

						const f32 t = intersect<analysis_mode>(view.nodes[node_id].bounds, r, inv_d);
						if (t < h.t)
						{
							// Same distance-sorted insertion the production kernel uses,
							// so the probe's counters are the honest counterfactual.
							size_t j = stack.size();
							stack.push_back(probe_entry{});
							CHECK_LE(stack.size(), probe_stack_limit);

							for (; j > insert_base; --j)
							{
								if (stack[j - 1].t > t) break;
								stack[j] = stack[j - 1];
							}

							stack[j].t = t;
							stack[j].ptr = view.nodes[node_id].ptr;
						}
					}
					out.stats.stack_depth(static_cast<u32>(stack.size()));
				}
				else
				{
					out.stats.prim_step();

					for (u32 i = 0; i < entry.ptr.prim_cnt; ++i)
					{
						const u32 slot = entry.ptr.prim_idx + i;
						out.stats.tri_test();
						if (prim(slot, r, h))
						{
							h.id = slot;
							out.hit = true;
						}
					}
				}
			}

			out.t = h.t;
			out.prim_slot = h.id;
			return out;
		}

		subtree_probe_result probe_any(const bvh2_view& view, const mesh_prims<analysis_mode>& prim,
			u32 child_node_id, const ray& r, f32 limit,
			std::vector<bvh_ptr>& stack)
		{
			subtree_probe_result out;
			out.t = limit;

			const vec3 inv_d = rcp(r.d);

			stack.clear();
			stack.push_back(view.nodes[child_node_id].ptr);

			while (!stack.empty())
			{
				const bvh_ptr ptr = stack.back();
				stack.pop_back();

				if (ptr.is_int)
				{
					out.stats.node_step();
					for (u32 i = 0; i < ptr.child_cnt; ++i)
					{
						const u32 node_id = ptr.child_idx + i;
						out.stats.box_test();
						if (intersect<analysis_mode>(view.nodes[node_id].bounds, r, inv_d) < limit)
						{
							stack.push_back(view.nodes[node_id].ptr);
							CHECK_LE(stack.size(), probe_stack_limit);
						}
					}
					out.stats.stack_depth(static_cast<u32>(stack.size()));
				}
				else
				{
					out.stats.prim_step();
					for (u32 i = 0; i < ptr.prim_cnt; ++i)
					{
						const u32 slot = ptr.prim_idx + i;
						hit h;
						h.t = limit;
						out.stats.tri_test();
						if (prim(slot, r, h))
						{
							out.hit = true;
							out.t = h.t;
							out.prim_slot = slot;
							return out;
						}
					}
				}
			}

			return out;
		}

		// ------------------------------------------------------------- scoring

		void set_score(candidate_scores& s, score_id id, double value, bool defined)
		{
			const u32 i = static_cast<u32>(id);
			s.value[i] = defined ? value : std::numeric_limits<double>::quiet_NaN();
			s.defined[i] = defined;
		}

		candidate_scores score_candidate(const directional_geometry& g, const aabb& child_box,
			const aabb& parent_box, const unit_direction& u, const directional_ratio& ratio)
		{
			candidate_scores s;

			set_score(s, score_id::directional, ratio.raw, ratio.valid);

			const double child_sa = double(child_box.surface_area());
			set_score(s, score_id::surface_density,
				child_sa > 0.0 ? g.triangle_surface_area_sum / child_sa : 0.0,
				child_sa > 0.0);

			set_score(s, score_id::primitive_count, double(g.primitive_count), true);

			const double parent_sa = double(parent_box.surface_area());
			set_score(s, score_id::box_surface_ratio,
				parent_sa > 0.0 ? child_sa / parent_sa : 0.0,
				parent_sa > 0.0);

			const double child_proj = projected_box_area(child_box, u);
			const double parent_proj = projected_box_area(parent_box, u);
			const bool proj_ok = u.valid && std::isfinite(child_proj) && std::isfinite(parent_proj)
				&& parent_proj > 0.0;
			set_score(s, score_id::box_projected_ratio,
				proj_ok ? child_proj / parent_proj : 0.0, proj_ok);

			// Node-geometry-only shapes. No ray direction enters these two; they
			// are exactly the quantities the collapse loss is built from.
			set_score(s, score_id::directional_mean_fill,
				directional_mean_fill(child_box, g), true);
			set_score(s, score_id::directional_min_fill,
				directional_min_fill(child_box, g, degenerate_axis_policy::exclude), true);

			// The rest of the (map x aggregate x weighting) grid. mean_fill above
			// is (clamp, weighted_mean, alpha=1); min_fill is (exponential, min).
			{
				fill_shape_args fa;
				fa.map = fill_map::clamp;
				fa.aggregate = fill_aggregate::weighted_mean;

				fa.weight_exponent = 0.0;
				set_score(s, score_id::fill_mean_clamp_a0, axis_fill_shape(child_box, g, fa), true);
				fa.weight_exponent = 0.5;
				set_score(s, score_id::fill_mean_clamp_a05, axis_fill_shape(child_box, g, fa), true);
				fa.weight_exponent = 2.0;
				set_score(s, score_id::fill_mean_clamp_a2, axis_fill_shape(child_box, g, fa), true);

				fa.map = fill_map::exponential;
				fa.weight_exponent = 1.0;
				set_score(s, score_id::fill_mean_exp, axis_fill_shape(child_box, g, fa), true);

				fa.map = fill_map::clamp;
				fa.aggregate = fill_aggregate::min;
				set_score(s, score_id::fill_min_clamp, axis_fill_shape(child_box, g, fa), true);
			}

			return s;
		}

		// Frozen rule: +1 correct, 0 tie (which includes "undefined for either
		// child"), -1 incorrect. Ties are never counted as half-correct.
		int rank_pair(const candidate_scores& hit_child, const candidate_scores& miss_child, u32 s)
		{
			if (!hit_child.defined[s] || !miss_child.defined[s]) return 0;
			if (hit_child.value[s] > miss_child.value[s]) return 1;
			if (hit_child.value[s] == miss_child.value[s]) return 0;
			return -1;
		}

		struct parent_candidate
		{
			u32               node_id{ 0 };
			f32               box_t{ 0.0f };
			bool              relevant_hit{ false };
			directional_ratio ratio{};
			candidate_scores  scores{};
			trace_stats       probe{};
		};

		// ------------------------------------------------------- analysis state

		struct analysis_context
		{
			const bvh2* tree{ nullptr };
			const mesh* m{ nullptr };
			const std::vector<directional_geometry>* geom{ nullptr };
			bvh2_view view{};
			mesh_prims<analysis_mode> prim{};
			directional_analysis_args args{};

			probe_scratch scratch;

			// E4: depth of every node, so a pair can be attributed to the level of
			// the tree it was decided at. Built once per rayset, not per ray.
			std::vector<u32> depth;

			directional_totals* totals{ nullptr };
			std::vector<candidate_event>* events{ nullptr };

			// Reset per ray.
			u32 ray_index{ 0 };
			u32 parent_visit{ 0 };
			ray_pair_record row{};

			void begin_ray(u32 index)
			{
				ray_index = index;
				parent_visit = 0;
				row = ray_pair_record{};
				row.ray_index = index;
			}
		};

		// Handles one parent visit: score every box-hit child, probe it
		// independently, bin it, and form the within-parent discordant pairs.
		void record_parent(analysis_context& ctx, u32 parent_node_id, const ray& r,
			f32 snapshot_limit, parent_candidate* candidates, u32 count)
		{
			directional_totals& totals = *ctx.totals;

			++totals.parent_visits;
			if (count == 0)
			{
				++ctx.parent_visit;
				return;
			}

			const unit_direction u = normalize_direction(r.d);

			const aabb parent_box = (parent_node_id == invalid_id)
				? ctx.view.nodes[0].bounds                  // pseudo-root entry
				: ctx.view.nodes[parent_node_id].bounds;

			// The pseudo-root entry stands for the root itself, which is depth 0.
			const u32 parent_depth = (parent_node_id == invalid_id
				|| parent_node_id >= ctx.depth.size()) ? 0u : ctx.depth[parent_node_id];
			const u32 depth_bucket = parent_depth < depth_bucket_count
				? parent_depth : depth_bucket_count - 1u;

			for (u32 i = 0; i < count; ++i)
			{
				parent_candidate& c = candidates[i];

				const aabb& child_box = ctx.view.nodes[c.node_id].bounds;
				const directional_geometry& g = (*ctx.geom)[c.node_id];

				c.ratio = compute_directional_ratio(g, child_box, u);
				c.scores = score_candidate(g, child_box, parent_box, u, c.ratio);

				if (ctx.args.query_kind == probe_query_kind::closest_improves_incumbent)
				{
					const subtree_probe_result p = probe_closest(ctx.view, ctx.prim, c.node_id, r,
						c.box_t, snapshot_limit, ctx.scratch.closest_stack);
					c.relevant_hit = p.hit;
					c.probe = p.stats;
				}
				else
				{
					const subtree_probe_result p = probe_any(ctx.view, ctx.prim, c.node_id, r,
						snapshot_limit, ctx.scratch.any_stack);
					c.relevant_hit = p.hit;
					c.probe = p.stats;
				}

				const u32 bin = static_cast<u32>(classify_ratio(c.ratio));

				++totals.candidate_events;
				++totals.bins.candidate_count[bin];

				if (bin == static_cast<u32>(ratio_bin::invalid))  ++totals.invalid_events;
				if (bin == static_cast<u32>(ratio_bin::raw_gt_1)) ++totals.saturated_events;

				totals.bins.probe_node_steps[bin] += c.probe.node_steps;
				totals.bins.probe_prim_steps[bin] += c.probe.prim_steps;
				totals.bins.probe_box_tests[bin] += c.probe.box_tests;
				totals.bins.probe_tri_tests[bin] += c.probe.tri_tests;

				if (c.relevant_hit)
				{
					++totals.relevant_hit_events;
					++totals.bins.relevant_hit_count[bin];
				}
				else
				{
					++totals.false_positive_events;
					++totals.bins.false_positive_count[bin];

					totals.bins.false_positive_node_steps[bin] += c.probe.node_steps;
					totals.bins.false_positive_prim_steps[bin] += c.probe.prim_steps;
					totals.bins.false_positive_box_tests[bin] += c.probe.box_tests;
					totals.bins.false_positive_tri_tests[bin] += c.probe.tri_tests;
				}

				++ctx.row.candidate_events;

				if (ctx.events)
				{
					candidate_event e;
					e.ray_index = ctx.ray_index;
					e.parent_visit = ctx.parent_visit;
					e.parent_node_id = parent_node_id;
					e.node_id = c.node_id;
					e.box_t = c.box_t;
					e.snapshot_limit = snapshot_limit;
					e.relevant_hit = c.relevant_hit;
					e.ratio = c.ratio;
					e.scores = c.scores;
					e.probe = c.probe;
					ctx.events->push_back(e);
				}
			}

			// Within-parent discordant sibling pairs.
			for (u32 i = 0; i < count; ++i)
			{
				for (u32 j = i + 1; j < count; ++j)
				{
					if (candidates[i].relevant_hit == candidates[j].relevant_hit) continue;

					const parent_candidate& hit_child = candidates[i].relevant_hit ? candidates[i] : candidates[j];
					const parent_candidate& miss_child = candidates[i].relevant_hit ? candidates[j] : candidates[i];

					++totals.discordant_pairs;
					++ctx.row.discordant_pairs;
					++totals.depths.pairs[depth_bucket];

					for (u32 s = 0; s < score_count; ++s)
					{
						const int verdict = rank_pair(hit_child.scores, miss_child.scores, s);
						if (verdict > 0)
						{
							++totals.correct[s];
							++ctx.row.correct[s];
							++totals.depths.correct[depth_bucket][s];
						}
						else if (verdict == 0)
						{
							++totals.ties[s];
							++ctx.row.ties[s];
							++totals.depths.ties[depth_bucket][s];
						}
					}
				}
			}

			++ctx.parent_visit;
		}

		// ----------------------------------------- production-mirroring kernels

		// A literal mirror of intersect() in core/traverse_bvh2.h, with node ids
		// carried in the stack entries and a probe hook at each parent visit. The
		// hook never touches h, the stack or stats.
		bool analysis_intersect(analysis_context& ctx, const ray& r, hit& h, trace_stats& stats)
		{
			const bvh2_view& bvh = ctx.view;
			if (bvh.node_count == 0) return false;

			const vec3 inv_d = rcp(r.d);

			struct stack_entry
			{
				f32     t;
				bvh_ptr ptr;
				u32     node_id;
			};

			stack_entry stack[bvh2_stack_size];
			u32         stack_size = 1u;

			stack[0].t = r.t_min;
			stack[0].ptr.is_int = 1;
			stack[0].ptr.child_cnt = 1;
			stack[0].ptr.child_idx = 0;
			stack[0].node_id = invalid_id;

			parent_candidate candidates[max_children];

			bool found_hit = false;
			do
			{
				const stack_entry entry = stack[--stack_size];
				if (entry.t >= h.t) continue;

				if (entry.ptr.is_int)
				{
					stats.node_step();

					// h.t cannot move inside the child loop, so this is the limit
					// every child of this visit is judged against.
					const f32 snapshot_limit = h.t;
					u32       candidate_count = 0;

					const u32 max_insert_depth = stack_size;

					for (u32 i = 0; i < entry.ptr.child_cnt; ++i)
					{
						const u32 node_id = entry.ptr.child_idx + i;
						stats.box_test();

						const f32 t = intersect<analysis_mode>(bvh.nodes[node_id].bounds, r, inv_d);
						if (t < h.t)
						{
							candidates[candidate_count].node_id = node_id;
							candidates[candidate_count].box_t = t;
							++candidate_count;

							u32 j = stack_size++;
							for (; j > max_insert_depth; --j)
							{
								if (stack[j - 1].t > t) break;
								stack[j] = stack[j - 1];
							}

							stack[j].t = t;
							stack[j].ptr = bvh.nodes[node_id].ptr;
							stack[j].node_id = node_id;
						}
					}
					stats.stack_depth(stack_size);

					if (ctx.args.enable_probes)
						record_parent(ctx, entry.node_id, r, snapshot_limit, candidates, candidate_count);
					else
						++ctx.totals->parent_visits;
				}
				else
				{
					stats.prim_step();

					for (u32 i = 0; i < entry.ptr.prim_cnt; ++i)
					{
						const u32 prim_id = entry.ptr.prim_idx + i;
						stats.tri_test();
						if (ctx.prim(prim_id, r, h))
						{
							h.id = prim_id;
							found_hit = true;
						}
					}
				}
			} while (stack_size);

			return found_hit;
		}

		// A literal mirror of occluded() in core/traverse_bvh2.h.
		bool analysis_occluded(analysis_context& ctx, const ray& r, trace_stats& stats)
		{
			const bvh2_view& bvh = ctx.view;
			if (bvh.node_count == 0) return false;

			const vec3 inv_d = rcp(r.d);

			struct stack_entry
			{
				bvh_ptr ptr;
				u32     node_id;
			};

			stack_entry stack[bvh2_stack_size];
			u32         stack_size = 1u;

			stack[0].ptr.is_int = 1;
			stack[0].ptr.child_cnt = 1;
			stack[0].ptr.child_idx = 0;
			stack[0].node_id = invalid_id;

			parent_candidate candidates[max_children];

			do
			{
				const stack_entry entry = stack[--stack_size];

				if (entry.ptr.is_int)
				{
					stats.node_step();

					u32 candidate_count = 0;

					for (u32 i = 0; i < entry.ptr.child_cnt; ++i)
					{
						const u32 node_id = entry.ptr.child_idx + i;
						stats.box_test();
						const f32 t = intersect<analysis_mode>(bvh.nodes[node_id].bounds, r, inv_d);
						if (t < r.t_max)
						{
							candidates[candidate_count].node_id = node_id;
							candidates[candidate_count].box_t = t;
							++candidate_count;

							stack[stack_size].ptr = bvh.nodes[node_id].ptr;
							stack[stack_size].node_id = node_id;
							++stack_size;
						}
					}
					stats.stack_depth(stack_size);

					if (ctx.args.enable_probes)
						record_parent(ctx, entry.node_id, r, r.t_max, candidates, candidate_count);
					else
						++ctx.totals->parent_visits;
				}
				else
				{
					stats.prim_step();
					for (u32 i = 0; i < entry.ptr.prim_cnt; ++i)
					{
						hit h;
						h.t = r.t_max;
						stats.tri_test();
						if (ctx.prim(entry.ptr.prim_idx + i, r, h)) return true;
					}
				}
			} while (stack_size);

			return false;
		}

	} // namespace

	const char* to_string(probe_query_kind k)
	{
		switch (k)
		{
		case probe_query_kind::closest_improves_incumbent: return "closest_improves_incumbent";
		case probe_query_kind::any_occluder_in_range:      return "any_occluder_in_range";
		default:                                          return "unknown";
		}
	}

	probe_query_kind query_kind_for(ray_distribution d)
	{
		return d == ray_distribution::shadow_ao
			? probe_query_kind::any_occluder_in_range
			: probe_query_kind::closest_improves_incumbent;
	}

	const char* to_string(score_id s)
	{
		switch (s)
		{
		case score_id::directional:         return "directional";
		case score_id::surface_density:     return "surface_density";
		case score_id::primitive_count:     return "primitive_count";
		case score_id::box_surface_ratio:   return "box_surface_ratio";
		case score_id::box_projected_ratio: return "box_projected_ratio";
		case score_id::directional_mean_fill: return "directional_mean_fill";
		case score_id::directional_min_fill:  return "directional_min_fill";
		case score_id::fill_mean_clamp_a0:  return "fill_mean_clamp_a0";
		case score_id::fill_mean_clamp_a05: return "fill_mean_clamp_a05";
		case score_id::fill_mean_clamp_a2:  return "fill_mean_clamp_a2";
		case score_id::fill_mean_exp:       return "fill_mean_exp";
		case score_id::fill_min_clamp:      return "fill_min_clamp";
		default:                            return "unknown";
		}
	}

	subtree_probe_result probe_subtree_closest(const bvh2& tree, const mesh& m,
		u32 child_node_id, const ray& r, f32 entry_t, f32 limit)
	{
		std::vector<probe_entry> stack;
		const auto prim = make_prims<analysis_mode>(m, tree);
		return probe_closest(tree.view(), prim, child_node_id, r, entry_t, limit, stack);
	}

	subtree_probe_result probe_subtree_any(const bvh2& tree, const mesh& m,
		u32 child_node_id, const ray& r, f32 limit)
	{
		std::vector<bvh_ptr> stack;
		const auto prim = make_prims<analysis_mode>(m, tree);
		return probe_any(tree.view(), prim, child_node_id, r, limit, stack);
	}

	void collect_descendant_slots(const bvh2& tree, u32 node_id, std::vector<u32>& out)
	{
		out.clear();

		std::vector<u32> stack;
		stack.push_back(node_id);

		while (!stack.empty())
		{
			const u32 id = stack.back();
			stack.pop_back();

			const bvh2_node& node = tree.nodes()[id];
			if (node.ptr.is_int)
			{
				for (u32 c = 0; c < node.ptr.child_cnt; ++c) stack.push_back(node.ptr.child_idx + c);
			}
			else
			{
				for (u32 p = 0; p < node.ptr.prim_cnt; ++p) out.push_back(node.ptr.prim_idx + p);
			}
		}

		// Slot order, so the reference scan is independent of tree shape.
		for (size_t i = 1; i < out.size(); ++i)
		{
			const u32 key = out[i];
			size_t j = i;
			while (j > 0 && out[j - 1] > key) { out[j] = out[j - 1]; --j; }
			out[j] = key;
		}
	}

	subtree_probe_result probe_slots_brute_force(const mesh& m, const bvh2& tree,
		const std::vector<u32>& slots, const ray& r, f32 limit, probe_query_kind kind)
	{
		subtree_probe_result out;
		out.t = limit;

		if (kind == probe_query_kind::any_occluder_in_range)
		{
			for (u32 slot : slots)
			{
				hit h;
				h.t = limit;
				if (intersect<analysis_mode>(m.get_triangle(tree.prim_index(slot)), r, h))
				{
					out.hit = true;
					out.t = h.t;
					out.prim_slot = slot;
					return out;
				}
			}
			return out;
		}

		hit h;
		h.t = limit;
		for (u32 slot : slots)
		{
			if (intersect<analysis_mode>(m.get_triangle(tree.prim_index(slot)), r, h))
			{
				h.id = slot;
				out.hit = true;
			}
		}
		out.t = h.t;
		out.prim_slot = h.id;
		return out;
	}

	directional_analysis_result analyze_rayset(const bvh2& tree, const mesh& m,
		const std::vector<directional_geometry>& geom,
		const rayset& rays,
		const directional_analysis_args& args)
	{
		CHECK(!tree.empty());
		CHECK_EQ(geom.size(), tree.nodes().size());

		directional_analysis_result result;

		analysis_context ctx;
		ctx.tree = &tree;
		ctx.m = &m;
		ctx.geom = &geom;
		ctx.view = tree.view();
		ctx.prim = make_prims<analysis_mode>(m, tree);
		ctx.args = args;
		ctx.totals = &result.totals;
		ctx.events = args.collect_events ? &result.events : nullptr;

		// Node depths, for the E4 per-level breakdown. Nodes are stored parent
		// before child, so one forward pass suffices.
		{
			const std::vector<bvh2_node>& nodes = tree.nodes();
			ctx.depth.assign(nodes.size(), 0u);
			for (size_t i = 0; i < nodes.size(); ++i)
			{
				if (!nodes[i].ptr.is_int) continue;
				for (u32 c = 0; c < nodes[i].ptr.child_cnt; ++c)
					ctx.depth[nodes[i].ptr.child_idx + c] = ctx.depth[i] + 1u;
			}
		}

		const u32 n = rays.size();
		result.totals.rays = n;

		if (args.collect_ray_hits)
		{
			if (args.query_kind == probe_query_kind::any_occluder_in_range) result.ray_occluded.reserve(n);
			else                                                           result.ray_hits.reserve(n);
		}

		for (u32 i = 0; i < n; ++i)
		{
			const ray r = rays.get(i);
			ctx.begin_ray(i);

			trace_stats stats;

			if (args.query_kind == probe_query_kind::any_occluder_in_range)
			{
				const bool occluded_ray = analysis_occluded(ctx, r, stats);
				if (occluded_ray) ++result.totals.trace_hits;
				if (args.collect_ray_hits) result.ray_occluded.push_back(occluded_ray ? 1u : 0u);
			}
			else
			{
				hit h;
				h.t = r.t_max;
				analysis_intersect(ctx, r, h, stats);
				if (h.valid()) ++result.totals.trace_hits;
				if (args.collect_ray_hits) result.ray_hits.push_back(h);
			}

			result.totals.trace_node_steps += stats.node_steps;
			result.totals.trace_prim_steps += stats.prim_steps;
			result.totals.trace_box_tests += stats.box_tests;
			result.totals.trace_tri_tests += stats.tri_tests;
			if (stats.max_stack > result.totals.trace_max_stack)
				result.totals.trace_max_stack = stats.max_stack;

			if (ctx.row.discordant_pairs > 0)
			{
				++result.totals.rays_with_pairs;
				result.totals.candidate_events_in_pair_rows += ctx.row.candidate_events;
				if (args.collect_pair_rows) result.pair_rows.push_back(ctx.row);
			}
		}

		return result;
	}

} // namespace bvh
