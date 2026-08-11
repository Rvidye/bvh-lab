#include <build/bvh2_builder.h>

#include <util/check.h>
#include <util/log.h>
#include <util/timer.h>

#include <algorithm>
#include <deque>

namespace bvh
{
	const char* to_string(split_method m)
	{
		switch (m)
		{
			case bvh::split_method::median: return "median";
			case bvh::split_method::sweep_sah: return "sweep_sah";
			case bvh::split_method::binned_sah: return "binned_sah";
			case bvh::split_method::binned_sah_arches: return "binned_sah_arches";
			default: return "unknown";
		}
	}

	namespace
	{
		struct build_object
		{
			aabb bounds{};
			f32 cost{ 1.0f };
			u32 index{ invalid_id };
		};

		struct build_event
		{
			u32 start;
			u32 end;
			u32 node_index;
			u32 depth;
		};

		// median split
		u32 split_median(std::vector<build_object>& objs, u32 start, u32 end)
		{
			aabb centroid_bounds;
			for (u32 i = start; i < end; ++i)
				centroid_bounds.add(objs[i].bounds.centroid());

			const u32 axis = centroid_bounds.longest_axis();
			const u32 mid = (start + end) / 2;

			std::nth_element(objs.begin() + start, objs.begin() + mid, objs.begin() + end,
				[axis](const build_object& a, const build_object& b) {
					return a.bounds.centroid()[axis] < b.bounds.centroid()[axis];
				});

			return mid;

		}

		// sweep_sah
		u32 split_sweep_sah(std::vector<build_object>& objs, u32 start, u32 end, std::vector<f32>& cost_left, std::vector<f32>& cost_right)
		{
			const u32 size = end - start;

			f32 best_sah = INFINITY;
			u32 best_split_axis = 0;
			u32 best_split_index = 0;

			cost_left.resize(size);
			cost_right.resize(size);

			for (u32 axis = 0; axis < 3; ++axis)
			{
				std::sort(objs.begin() + start, objs.begin() + end,
					[axis](const build_object& a, const build_object& b) { return a.bounds.centroid()[axis] < b.bounds.centroid()[axis]; });

				const build_object* o = objs.data() + start;

				aabb left_bounds;
				f32  left_cost_sum = 0.0f;
				for (u32 i = 0; i < size; ++i)
				{
					cost_left[i] = left_cost_sum * left_bounds.surface_area();
					left_bounds.add(o[i].bounds);
					left_cost_sum += o[i].cost;
				}
				cost_left[0] = 0.0f;

				aabb right_bounds;
				f32  right_cost_sum = 0.0f;
				for (i32 i = static_cast<i32>(size) - 1; i >= 0; --i)
				{
					right_cost_sum += o[i].cost;
					right_bounds.add(o[i].bounds);
					cost_right[i] = right_cost_sum * right_bounds.surface_area();
				}

				for (u32 i = 1; i < size; ++i)
				{
					const f32 cost = cost_left[i] + cost_right[i];
					if (cost < best_sah)
					{
						best_sah = cost;
						best_split_axis = axis;
						best_split_index = i;
					}
				}
			}

			// Re-sort on the winning axis: the array is currently ordered by axis 2.
			std::sort(objs.begin() + start, objs.begin() + end,
				[best_split_axis](const build_object& a, const build_object& b) {
					return a.bounds.centroid()[best_split_axis] < b.bounds.centroid()[best_split_axis];
				});

			if (best_sah == INFINITY) return (start + end) / 2;
			return start + best_split_index;
		}

		// binned_sah
		u32 partition_on_plane(std::vector<build_object>& objs, u32 start, u32 end, u32 axis, f32 pos, f32& cost)
		{
			aabb left_bounds, right_bounds;
			f32 left_cost = 0.0f, right_cost = 0.0f;

			u32 i = start, j = end;
			while (i < j)
			{
				const build_object& o = objs[i];
				if (o.bounds.centroid()[axis] <= pos)
				{
					++i;
					left_cost += o.cost;
					left_bounds.add(o.bounds);
				}
				else
				{
					--j;
					std::swap(objs[i], objs[j]);
					right_cost += o.cost;
					right_bounds.add(o.bounds);
				}
			}

			if (i == start || i == end)
			{
				cost = INFINITY;
				return (start + end) / 2;
			}

			cost = left_cost * left_bounds.surface_area() + right_cost * right_bounds.surface_area();
			return i;
		}

		u32 split_binned_sah_arches(std::vector<build_object>& objs, u32 start, u32 end, u32 bins)
		{
			aabb bounds;
			for (u32 i = start; i < end; ++i) bounds.add(objs[i].bounds);

			f32 best_sah = INFINITY;
			u32 best_axis = 0;
			f32 best_pos = 0.0f;

			for (u32 axis = 0; axis < 3; ++axis)
			{
				for (u32 b = 1; b < bins; ++b)
				{
					const f32 ratio = static_cast<f32>(b) / static_cast<f32>(bins);
					const f32 pos = (1.0f - ratio) * bounds.min[axis] + ratio * bounds.max[axis];

					f32 sah;
					partition_on_plane(objs, start, end, axis, pos, sah);
					if (sah < best_sah)
					{
						best_axis = axis;
						best_sah = sah;
						best_pos = pos;
					}
				}
			}

			if (best_sah == INFINITY) return (start + end) / 2;

			f32 ignored;
			return partition_on_plane(objs, start, end, best_axis, best_pos, ignored);
		}

		// Wald 2007 binned_sah
		struct bin
		{
			aabb bounds{};
			u32  count{ 0 };
		};

		u32 split_binned_sah(std::vector<build_object>& objs, u32 start, u32 end, u32 bin_count,
			std::vector<bin>& bins,
			std::vector<f32>& area_left, std::vector<u32>& count_left)
		{
			aabb centroid_bounds;
			for (u32 i = start; i < end; ++i) centroid_bounds.add(objs[i].bounds.centroid());

			f32 best_sah = INFINITY;
			u32 best_axis = 0;
			u32 best_bin = 0;

			bins.resize(bin_count);
			area_left.resize(bin_count);
			count_left.resize(bin_count);

			for (u32 axis = 0; axis < 3; ++axis)
			{
				const f32 lo = centroid_bounds.min[axis];
				const f32 extent = centroid_bounds.max[axis] - lo;
				if (extent <= 0.0f) continue; // no separation on this axis

				// Scale chosen so the maximum centroid lands in the last bin rather
				// than one past it.
				const f32 scale = static_cast<f32>(bin_count) * (1.0f - 1e-6f) / extent;

				for (bin& b : bins) b = bin{};

				for (u32 i = start; i < end; ++i)
				{
					const u32 idx = static_cast<u32>((objs[i].bounds.centroid()[axis] - lo) * scale);
					bins[idx].bounds.add(objs[i].bounds);
					bins[idx].count += 1;
				}

				// Forward sweep: area and count of everything left of each boundary.
				{
					aabb acc;
					u32  n = 0;
					for (u32 b = 0; b < bin_count; ++b)
					{
						area_left[b] = acc.surface_area();
						count_left[b] = n;
						acc.add(bins[b].bounds);
						n += bins[b].count;
					}
				}

				// Backward sweep, evaluating each boundary as it becomes known.
				{
					aabb acc;
					u32  n = 0;
					for (u32 b = bin_count; b-- > 1;)
					{
						acc.add(bins[b].bounds);
						n += bins[b].count;

						if (count_left[b] == 0 || n == 0) continue;

						const f32 sah = static_cast<f32>(count_left[b]) * area_left[b]
							+ static_cast<f32>(n) * acc.surface_area();

						if (sah < best_sah)
						{
							best_sah = sah;
							best_axis = axis;
							best_bin = b;
						}
					}
				}
			}

			if (best_sah == INFINITY) return (start + end) / 2;

			// Partition against the winning boundary.
			const f32 lo = centroid_bounds.min[best_axis];
			const f32 extent = centroid_bounds.max[best_axis] - lo;
			const f32 scale = static_cast<f32>(bin_count) * (1.0f - 1e-6f) / extent;

			const auto mid = std::partition(objs.begin() + start, objs.begin() + end,
				[&](const build_object& o) {
					const u32 idx = static_cast<u32>((o.bounds.centroid()[best_axis] - lo) * scale);
					return idx < best_bin;
				});

			const u32 split = static_cast<u32>(mid - objs.begin());
			if (split == start || split == end) return (start + end) / 2;
			return split;
		}
	}// namespace

	void bvh2::build(const mesh& m, const build_args& args)
	{
		CHECK(!m.empty());

		_args = args;
		_width = 2;
		_nodes.clear();
		_prim_indices.clear();

		timer t;

		const u32 count = m.triangle_count();

		std::vector<build_object> objs(count);
		for (u32 i = 0; i < count; ++i)
		{
			objs[i].bounds = m.triangle_bounds(i);
			objs[i].cost = 1.0f;
			objs[i].index = i;
		}

		// Scratch reused across the whole build so the split routines do not allocate per node.
		std::vector<f32> cost_left, cost_right, area_left;
		std::vector<u32> count_left;
		std::vector<bin> bins;

		// Breadth-first via a queue
		// BFS is what guarantees children always land at a HIGHER index than their parent.
		std::deque<build_event> queue;
		queue.push_back(build_event{ 0u, count, 0u, 0u });

		_nodes.reserve(size_t(count) * 2 - 1);
		_nodes.emplace_back();

		u32 max_depth = 0;

		while (!queue.empty())
		{
			const build_event event = queue.front();
			queue.pop_front();

			max_depth = max(max_depth, event.depth);

			const u32 size = event.end - event.start;

			u32 split = invalid_id;
			if (size > args.max_leaf_size)
			{
				switch (args.method)
				{
				case split_method::median:
					split = split_median(objs, event.start, event.end);
					break;
				case split_method::sweep_sah:
					split = split_sweep_sah(objs, event.start, event.end, cost_left, cost_right);
					break;
				case split_method::binned_sah:
					split = split_binned_sah(objs, event.start, event.end, args.bins,
						bins, area_left, count_left);
					break;
				case split_method::binned_sah_arches:
					split = split_binned_sah_arches(objs, event.start, event.end, args.bins);
					break;
				}

				// A split that puts everything on one side makes no progress and
				// would recurse forever.
				if (split <= event.start || split >= event.end) split = invalid_id;
			}

			if (split != invalid_id)
			{
				bvh2_node& node = _nodes[event.node_index];
				node.ptr.is_int = 1;
				node.ptr.child_cnt = 2;
				node.ptr.child_idx = static_cast<u32>(_nodes.size());

				queue.push_back(build_event{ event.start, split, static_cast<u32>(_nodes.size()) + 0u, event.depth + 1u });
				queue.push_back(build_event{ split, event.end, static_cast<u32>(_nodes.size()) + 1u, event.depth + 1u });

				_nodes.emplace_back();
				_nodes.emplace_back();
			}
			else
			{
				// The 5-bit count field caps a leaf at 31 primitives.
				CHECK_LE(size, 31u);
				bvh2_node& node = _nodes[event.node_index];
				node.ptr.is_int = 0;
				node.ptr.prim_cnt = size;
				node.ptr.prim_idx = event.start;
			}
		}

		// The permutation the builder settled on: slot -> original triangle id.
		_prim_indices.resize(count);
		for (u32 i = 0; i < count; ++i) _prim_indices[i] = objs[i].index;

		// Bounds are filled bottom-up once the topology is final
		refit(m);

		_report = compute_report();
		_report.build_ms = t.elapsed_ms();
		DCHECK_EQ(_report.max_depth, max_depth);

		if (!args.silent)
		{
			LOG_INFO("bvh2[%s]: %u nodes (%u leaf, %u interior), depth %u, %.1f ms",
				to_string(args.method), _report.node_count, _report.leaf_count,
				_report.interior_count, _report.max_depth, _report.build_ms);
		}
	}

	void bvh2::apply_reorder(mesh& m)
	{
		CHECK_EQ(_prim_indices.size(), size_t(m.triangle_count()));
		m.reorder(_prim_indices);
		// After permuting the mesh, leaf ranges index it directly.
		for (u32 i = 0; i < _prim_indices.size(); ++i) _prim_indices[i] = i;
	}

	void bvh2::refit(const mesh& m)
	{
		if (_nodes.empty()) return;
		for (i32 i = static_cast<i32>(_nodes.size()) - 1; i >= 0; --i)
		{
			bvh2_node& node = _nodes[i];
			node.bounds = aabb{};

			if (node.ptr.is_int)
			{
				for (u32 c = 0; c < node.ptr.child_cnt; ++c)
				{
					DCHECK_GT(node.ptr.child_idx + c, u32(i));
					node.bounds.add(_nodes[node.ptr.child_idx + c].bounds);
				}
			}
			else
			{
				for (u32 p = 0; p < node.ptr.prim_cnt; ++p)
					node.bounds.add(m.triangle_bounds(_prim_indices[node.ptr.prim_idx + p]));
			}
		}
	}

	build_report bvh2::compute_report() const
	{
		build_report r;
		r.node_count = static_cast<u32>(_nodes.size());

		u64 total_leaf_prims = 0;
		for (const bvh2_node& node : _nodes)
		{
			if (node.ptr.is_int)
			{
				++r.interior_count;
			}
			else
			{
				++r.leaf_count;
				total_leaf_prims += node.ptr.prim_cnt;
				r.max_leaf_size = max(r.max_leaf_size, u32(node.ptr.prim_cnt));
			}
		}

		r.mean_leaf_size = r.leaf_count ? double(total_leaf_prims) / double(r.leaf_count) : 0.0;
		r.build_ms = _report.build_ms;
		if (!_nodes.empty())
		{
			std::vector<u32> depth(_nodes.size(), 0u);
			for (u32 i = 0; i < _nodes.size(); ++i)
			{
				r.max_depth = max(r.max_depth, depth[i]);
				if (!_nodes[i].ptr.is_int) continue;
				for (u32 c = 0; c < _nodes[i].ptr.child_cnt; ++c)
					depth[_nodes[i].ptr.child_idx + c] = depth[i] + 1;
			}
		}

		return r;
	}

	void bvh2::replace_nodes(std::vector<bvh2_node>&& nodes, u32 width)
	{
		CHECK(!nodes.empty());
		_nodes = std::move(nodes);
		_width = width;

		for (u32 i = 0; i < _nodes.size(); ++i)
		{
			if (!_nodes[i].ptr.is_int) continue;
			for (u32 c = 0; c < _nodes[i].ptr.child_cnt; ++c)
				CHECK_GT(_nodes[i].ptr.child_idx + c, i);
		}

		_report = compute_report();
	}

} // namespace bvh


