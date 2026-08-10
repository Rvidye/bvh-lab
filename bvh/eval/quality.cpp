#include <eval/quality.h>

#include <util/check.h>
#include <util/parallel.h>
#include <util/timer.h>

#include <atomic>
#include <vector>

namespace bvh {

	namespace {

		constexpr u32 max_clip_verts = 16; // 3 + one vertex per clip plane, with slack

		struct polygon
		{
			vec3 v[max_clip_verts];
			u32  n{ 0 };

			void push(const vec3& p) { if (n < max_clip_verts) v[n++] = p; }
		};

		// Clips to the half-space { p : p[axis] <= limit } when keep_lower, else { p : p[axis] >= limit }.
		void clip_to_plane(polygon& poly, u32 axis, f32 limit, bool keep_lower)
		{
			if (poly.n == 0) return;

			polygon out;
			for (u32 i = 0; i < poly.n; ++i)
			{
				const vec3& a = poly.v[i];
				const vec3& b = poly.v[(i + 1) % poly.n];

				const f32 da = keep_lower ? (limit - a[axis]) : (a[axis] - limit);
				const f32 db = keep_lower ? (limit - b[axis]) : (b[axis] - limit);

				const bool a_in = da >= 0.0f;
				const bool b_in = db >= 0.0f;

				if (a_in) out.push(a);

				if (a_in != b_in)
				{
					const f32 denom = da - db;
					if (denom != 0.0f) out.push(a + (b - a) * (da / denom));
				}
			}

			poly = out;
		}

		f32 polygon_area(const polygon& poly)
		{
			if (poly.n < 3) return 0.0f;

			vec3 n(0.0f);
			for (u32 i = 0; i < poly.n; ++i)
				n += cross(poly.v[i], poly.v[(i + 1) % poly.n]);

			return 0.5f * length(n);
		}

		f32 triangle_area_in_box(const triangle& tri, const aabb& box)
		{
			polygon poly;
			poly.push(tri.vrts[0]);
			poly.push(tri.vrts[1]);
			poly.push(tri.vrts[2]);

			for (u32 axis = 0; axis < 3; ++axis)
			{
				clip_to_plane(poly, axis, box.max[axis], true);
				clip_to_plane(poly, axis, box.min[axis], false);
				if (poly.n == 0) return 0.0f;
			}

			return polygon_area(poly);
		}

		f32 triangle_area(const triangle& tri)
		{
			return 0.5f * length(cross(tri.vrts[1] - tri.vrts[0], tri.vrts[2] - tri.vrts[0]));
		}

		bool boxes_overlap(const aabb& a, const aabb& b)
		{
			return a.min.x <= b.max.x && a.max.x >= b.min.x
				&& a.min.y <= b.max.y && a.max.y >= b.min.y
				&& a.min.z <= b.max.z && a.max.z >= b.min.z;
		}

		struct slot_range
		{
			u32 begin{ invalid_id };
			u32 end{ 0 };
		};

		std::vector<slot_range> compute_slot_ranges(const std::vector<bvh2_node>& nodes)
		{
			std::vector<slot_range> ranges(nodes.size());

			// Reverse order: children always have a higher index than their parent.
			for (i32 i = static_cast<i32>(nodes.size()) - 1; i >= 0; --i)
			{
				const bvh2_node& node = nodes[i];
				slot_range& r = ranges[i];

				if (node.ptr.is_int)
				{
					for (u32 c = 0; c < node.ptr.child_cnt; ++c)
					{
						const slot_range& cr = ranges[node.ptr.child_idx + c];
						r.begin = min(r.begin, cr.begin);
						r.end = max(r.end, cr.end);
					}
				}
				else
				{
					r.begin = node.ptr.prim_idx;
					r.end = node.ptr.prim_idx + node.ptr.prim_cnt;
				}
			}

			return ranges;
		}

	} // namespace

	quality_metrics evaluate(const bvh2& tree, const mesh& m, const quality_args& args)
	{
		quality_metrics q;

		const std::vector<bvh2_node>& nodes = tree.nodes();
		if (nodes.empty()) return q;

		const build_report r = tree.compute_report();
		q.node_count = r.node_count;
		q.leaf_count = r.leaf_count;
		q.interior_count = r.interior_count;
		q.max_depth = r.max_depth;
		q.mean_leaf_size = r.mean_leaf_size;
		q.bytes = nodes.size() * sizeof(bvh2_node);
		q.bytes_per_tri = m.triangle_count() ? double(q.bytes) / double(m.triangle_count()) : 0.0;

		const double root_area = double(nodes[0].bounds.surface_area());
		if (root_area <= 0.0) return q;

		// SAH cost
		// c_t * sum(A_i/A_root) over interior nodes
		// + c_i * sum((A_l/A_root) * n_l) over leaves
		double sah = 0.0, sah_arches = 0.0;
		for (const bvh2_node& node : nodes)
		{
			const double a = double(node.bounds.surface_area()) / root_area;

			if (node.ptr.is_int) sah += args.c_traversal * a;
			else                 sah += args.c_intersect * a * double(node.ptr.prim_cnt);
			sah_arches += a * 64.0;
		}
		q.sah_cost = sah;
		q.sah_cost_arches = sah_arches;

		if (!args.compute_epo) return q;

		// EPO
		// For each node, the surface area of geometry that overlaps its box but is
		// NOT in its subtree, weighted by that node's cost and normalised by total scene surface area.
		const std::vector<slot_range> ranges = compute_slot_ranges(nodes);
		const std::vector<u32>& slots = tree.prim_indices();
		const u32 count = m.triangle_count();

		std::vector<aabb> tri_bounds(count);
		std::vector<f32>  tri_area(count);
		double            total_area = 0.0;
		for (u32 s = 0; s < count; ++s)
		{
			const triangle t = m.get_triangle(slots[s]);
			tri_bounds[s] = t.bounds();
			tri_area[s] = triangle_area(t);
			total_area += double(tri_area[s]);
		}

		if (total_area <= 0.0) return q;

		std::atomic<u64> epo_bits{ 0 }; // accumulate as scaled integer to stay associative
		constexpr double epo_scale = 1e9;

		parallel_for(static_cast<u32>(nodes.size()), [&](u32 n) {
			const bvh2_node& node = nodes[n];
			const slot_range& sr = ranges[n];

			const double node_cost = node.ptr.is_int
				? args.c_traversal
				: args.c_intersect * double(node.ptr.prim_cnt);
			if (node_cost == 0.0) return;

			double overlap = 0.0;
			for (u32 s = 0; s < count; ++s)
			{
				if (s >= sr.begin && s < sr.end) continue; // own subtree
				if (tri_area[s] <= 0.0f) continue;
				if (!boxes_overlap(tri_bounds[s], node.bounds)) continue;

				overlap += double(triangle_area_in_box(m.get_triangle(slots[s]), node.bounds));
			}

			if (overlap > 0.0)
			{
				const u64 scaled = static_cast<u64>(node_cost * overlap * epo_scale / total_area);
				epo_bits.fetch_add(scaled, std::memory_order_relaxed);
			}
			});

		q.epo = double(epo_bits.load()) / epo_scale;
		q.combined = (1.0 - args.epo_weight) * q.sah_cost + args.epo_weight * q.epo;

		return q;
	}

} // namespace bvh
