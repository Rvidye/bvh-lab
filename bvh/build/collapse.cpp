#include <build/collapse.h>

#include <util/check.h>
#include <util/log.h>
#include <util/timer.h>

#include <deque>
#include <vector>

namespace bvh
{
	const char* to_string(collapse_method m)
	{
		switch (m)
		{
			case bvh::collapse_method::greedy:				return "greedy";
			case bvh::collapse_method::dynamic_programming: return "dp";
			default:										return "unknown";
		}
	}

	namespace
	{
		// greedy collapse:
		// walk tree, at each surviving node repeatedly replace the set member with largest surface area by it's two children, until the set is 'width' wide
		// or nothing is left to expand.
		std::vector<bvh2_node> collapse_greedy(const std::vector<bvh2_node>& src, u32 width)
		{
			struct event { u32 src_index; u32 dst_index; };

			std::deque<event> queue;
			queue.push_back({ 0u, 0u });

			std::vector<bvh2_node> dst;
			dst.reserve(src.size());
			dst.emplace_back();

			std::vector<u32> node_set;
			node_set.reserve(width);

			while (!queue.empty())
			{
				const event e = queue.front();
				queue.pop_front();

				const bvh2_node& node = src[e.src_index];
				bvh2_node&		 out  = dst[e.dst_index];
				out = node;

				if (!node.ptr.is_int) continue;

				node_set.clear();
				node_set.push_back(e.src_index);

				while (node_set.size() < width)
				{
					f32 best_area = -INFINITY;
					u32 best_slot = invalid_id;

					for (u32 i = 0; i < node_set.size(); ++i)
					{
						if (!src[node_set[i]].ptr.is_int) continue;
						const f32 area = src[node_set[i]].bounds.surface_area();
						if (area <= best_area) continue;
						best_slot = i;
						best_area = area;
					}

					if (best_slot == invalid_id) break; // every member is a leaf

					const u32 base = src[node_set[best_slot]].ptr.child_idx;
					node_set[best_slot] = base;
					node_set.insert(node_set.begin() + best_slot + 1, base + 1);
				}
				out.ptr.child_idx = static_cast<u32>(dst.size());
				out.ptr.child_cnt = static_cast<u32>(node_set.size());

				for (u32 i = 0; i < node_set.size(); ++i)
				{
					queue.push_back({ node_set[i], static_cast<u32>(dst.size()) });
					dst.emplace_back();
				}
			}
			return dst;
		}

		// dynamic programming
		// for every node and every slot budget j in [1, width], compute the cheapest way to represent that subtree in j slots.
		// 1: Leaf: collapse entire subtree into one leaf
		// 2: Internal: spent one slot on this node, and split the remaining width between its two children
		// 3: spend no slot on this node, hand j slots straight through to the children, so this node disappeares into its parent

		enum decision_type : u8 { d_leaf, d_internal, d_distribute };

		struct decision
		{
			u8 type{ d_leaf };
			u8 dist_left{ 0 };
			u8 dist_right{ 0 };
			f32 cost{ INFINITY };
		};

		struct dp_cache
		{
			u32 prim_cnt{ 0 };
			u32 prim_idx{ 0 };
			decision decisions[max_collapse_width + 1];
		};

		std::vector<bvh2_node> collapse_dp(const std::vector<bvh2_node>& src, const bvh2& tree, const mesh& m, const collapse_args& args)
		{
			const u32 width = args.width;
			std::vector<dp_cache> cache(src.size());

			// bottom up, children always have a higher index that their parent
			for (i32 n = static_cast<i32>(src.size()) - 1; n >= 0; --n)
			{
				const bvh2_node& node = src[n];
				dp_cache& c = cache[n];

				if (node.ptr.is_int)
				{
					const u32 l = node.ptr.child_idx + 0;
					const u32 r = node.ptr.child_idx + 1;
					c.prim_cnt = cache[l].prim_cnt + cache[r].prim_cnt;
					c.prim_idx = cache[l].prim_idx;
				}
				else
				{
					c.prim_cnt = node.ptr.prim_cnt;
					c.prim_idx = node.ptr.prim_idx;
				}

				const f32 area = node.bounds.surface_area();

				// leaf cost
				// INFINITY when the leaf policy forrbits it, which is how a subtree is prevented from collapsing past max_leaf_size
				f32 leaf_cost = INFINITY;
				if (c.prim_cnt <= args.max_leaf_size)
					leaf_cost = args.c_intersect * static_cast<f32>(c.prim_cnt) * area;

				c.decisions[1].cost = leaf_cost;
				c.decisions[1].type = d_leaf;
				if (c.decisions[1].cost != c.decisions[1].cost) c.decisions[1].cost = INFINITY;

				if (node.ptr.is_int)
				{
					const u32 l = node.ptr.child_idx + 0;
					const u32 r = node.ptr.child_idx + 1;

					for (u32 k = 1; k < width; ++k)
					{
						const f32 cost = args.c_traversal * area + cache[l].decisions[k].cost + cache[r].decisions[width - k].cost;
						if (cost < c.decisions[1].cost)
						{
							c.decisions[1].dist_left = static_cast<u8>(k);
							c.decisions[1].dist_right = static_cast<u8>(width - k);
							c.decisions[1].cost = cost;
							c.decisions[1].type = d_internal;
						}
					}

					for (u32 j = 2; j < width; ++j)
					{
						c.decisions[j] = c.decisions[j - 1];
						for (u32 k = 1; k < j; ++k)
						{
							const f32 cost = cache[l].decisions[k].cost
								+ cache[r].decisions[j - k].cost;
							if (cost < c.decisions[j].cost)
							{
								c.decisions[j].dist_left = static_cast<u8>(k);
								c.decisions[j].dist_right = static_cast<u8>(j - k);
								c.decisions[j].cost = cost;
								c.decisions[j].type = d_distribute;
							}
						}
					}
				}
				else
				{
					for (u32 j = 2; j < width; ++j) c.decisions[j] = c.decisions[j - 1];
				}

				CHECK_MSG(c.decisions[1].cost != INFINITY, "collapse: node %d has no valid arrangement at width %u " "(max_leaf_size %u, %u prims)", n, width, args.max_leaf_size, c.prim_cnt);
			}

			// emit
			struct item { u32 node; u32 slot; };
			struct event { item it; u32 dst_index; };

			std::deque<event> queue;
			queue.push_back({ {0u, 1u}, 0u });

			std::vector<bvh2_node> dst;
			dst.reserve(src.size());
			dst.emplace_back();

			std::vector<item> node_set;
			node_set.reserve(width);

			while (!queue.empty())
			{
				const event e = queue.front();
				queue.pop_front();

				const bvh2_node& node = src[e.it.node];
				bvh2_node& out = dst[e.dst_index];
				out = node;

				const decision d = cache[e.it.node].decisions[e.it.slot];

				if (d.type == d_internal)
				{
					node_set.clear();
					node_set.push_back({ node.ptr.child_idx + 0, d.dist_left });
					node_set.push_back({ node.ptr.child_idx + 1, d.dist_right });

					// Expand every DISTRIBUTE in place until only INTERNAL and LEAF
					// remain; those are the children this wide node actually stores.
					for (u32 i = 0; i < node_set.size();)
					{
						const item     cur = node_set[i];
						const decision cd = cache[cur.node].decisions[cur.slot];

						if (cd.type == d_distribute)
						{
							const u32 base = src[cur.node].ptr.child_idx;
							node_set[i] = { base + 0, cd.dist_left };
							node_set.insert(node_set.begin() + i + 1, { base + 1, cd.dist_right });
						}
						else ++i;
					}

					CHECK_LE(node_set.size(), size_t(width));

					out.ptr.is_int = 1;
					out.ptr.child_idx = static_cast<u32>(dst.size());
					out.ptr.child_cnt = static_cast<u32>(node_set.size());

					for (const item& it : node_set)
					{
						queue.push_back({ it, static_cast<u32>(dst.size()) });
						dst.emplace_back();
					}
				}
				else if (d.type == d_leaf)
				{
					out.ptr.is_int = 0;
					out.ptr.prim_cnt = cache[e.it.node].prim_cnt;
					out.ptr.prim_idx = cache[e.it.node].prim_idx;
				}
				else
				{
					// A DISTRIBUTE can only be reached through the expansion loop above, never as a queued node.
					CHECK_MSG(false, "collapse: DISTRIBUTE reached as a queued node");
				}
			}
			return dst;
		}
	} // namespace

	collapse_report collapse(bvh2& tree, const mesh& m, const collapse_args& args)
	{
		CHECK(!tree.empty());
		CHECK_GE(args.width, 2u);
		CHECK_LE(args.width, max_collapse_width);

		timer t;

		const std::vector<bvh2_node>& src = tree.nodes();
		std::vector<bvh2_node> dst = (args.method == collapse_method::greedy) ? collapse_greedy(src, args.width) : collapse_dp(src, tree, m, args);

		tree.replace_nodes(std::move(dst), args.width);

		collapse_report r;
		r.collapse_ms = t.elapsed_ms();
		r.width = args.width;

		const std::vector<bvh2_node>& out = tree.nodes();
		r.node_count = static_cast<u32>(out.size());
		r.max_depth = tree.report().max_depth;

		u64 total_children = 0;
		for (const bvh2_node& node : out)
		{
			if (node.ptr.is_int)
			{
				++r.interior_count;
				total_children += node.ptr.child_cnt;
				r.fullness_histogram[bvh::min(u32(node.ptr.child_cnt), args.width)]++;
			}
			else
			{
				++r.leaf_count;
			}
		}

		r.mean_fullness = r.interior_count ? double(total_children) / double(r.interior_count) : 0.0;

		if (!args.silent)
		{
			LOG_INFO("collapse[%s w%u]: %u nodes (%u int, %u leaf), depth %u, fullness %.2f, %.1f ms", to_string(args.method), args.width, r.node_count, r.interior_count,  r.leaf_count, r.max_depth, r.mean_fullness, r.collapse_ms);
			for (u32 i = 2; i <= args.width; ++i)
			{
				const double pct = r.interior_count ? 100.0 * double(r.fullness_histogram[i]) / double(r.interior_count) : 0.0;
				LOG_INFO("  %2u children: %5.1f%% %.*s", i, pct, int(pct / 2.0 + 0.5), "..................................................");
			}
		}
		return r;
	}

	overlap_profile compute_overlap_profile(const bvh2& tree)
	{
		overlap_profile p;

		const std::vector<bvh2_node>& nodes = tree.nodes();
		if (nodes.empty()) return p;

		std::vector<u32> depth(nodes.size(), 0u);
		std::vector<double> sum(overlap_profile::max_depth_buckets, 0.0);

		for (u32 i = 0; i < nodes.size(); ++i)
		{
			const bvh2_node& node = nodes[i];
			const u32 d = depth[i];

			if (!node.ptr.is_int) continue;
			for (u32 c = 0; c < node.ptr.child_cnt; ++c)
				depth[node.ptr.child_idx + c] = d + 1;

			if (d >= overlap_profile::max_depth_buckets) continue;

			const f32 parent_area = node.bounds.surface_area();
			if (parent_area <= 0.0f) continue;

			f32 child_area = 0.0f;
			for (u32 c = 0; c < node.ptr.child_cnt; ++c)
				child_area += nodes[node.ptr.child_idx + c].bounds.surface_area();

			sum[d] += double(child_area) / double(parent_area);
			p.nodes[d]++;
			p.depth_count = bvh::max(p.depth_count, d + 1);
		}

		for (u32 d = 0; d < overlap_profile::max_depth_buckets; ++d)
			p.mean_overlap[d] = p.nodes[d] ? sum[d] / double(p.nodes[d]) : 0.0;

		return p;
	}
} // namespace bvh

