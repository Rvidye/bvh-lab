#include <eval/directional_geometry.h>

#include <util/check.h>

#include <cmath>
#include <limits>

namespace bvh
{
	namespace
	{
		constexpr double nan_value() { return std::numeric_limits<double>::quiet_NaN(); }

		inline bool finite(double v) { return std::isfinite(v); }

		// Box extents in double. An empty box (min > max) has no extent.
		inline void box_extent(const aabb& box, double& ex, double& ey, double& ez)
		{
			if (box.empty())
			{
				ex = ey = ez = 0.0;
				return;
			}
			ex = double(box.max.x) - double(box.min.x);
			ey = double(box.max.y) - double(box.min.y);
			ez = double(box.max.z) - double(box.min.z);
		}

	} // namespace

	unit_direction normalize_direction(const vec3& d)
	{
		unit_direction u;

		const double x = double(d.x);
		const double y = double(d.y);
		const double z = double(d.z);

		if (!finite(x) || !finite(y) || !finite(z)) return u;

		const double len_sq = x * x + y * y + z * z;
		if (!(len_sq > 0.0) || !finite(len_sq)) return u;

		const double len = std::sqrt(len_sq);
		if (!(len > 0.0) || !finite(len)) return u;

		u.x = x / len;
		u.y = y / len;
		u.z = z / len;
		u.valid = finite(u.x) && finite(u.y) && finite(u.z);
		return u;
	}

	directional_geometry triangle_directional_geometry(const triangle& t)
	{
		const double v0x = double(t.vrts[0].x), v0y = double(t.vrts[0].y), v0z = double(t.vrts[0].z);
		const double v1x = double(t.vrts[1].x), v1y = double(t.vrts[1].y), v1z = double(t.vrts[1].z);
		const double v2x = double(t.vrts[2].x), v2y = double(t.vrts[2].y), v2z = double(t.vrts[2].z);

		const double ax = v1x - v0x, ay = v1y - v0y, az = v1z - v0z;
		const double bx = v2x - v0x, by = v2y - v0y, bz = v2z - v0z;

		const double cx = ay * bz - az * by;
		const double cy = az * bx - ax * bz;
		const double cz = ax * by - ay * bx;

		directional_geometry g;
		g.p_yz = 0.5 * std::abs(cx);
		g.p_xz = 0.5 * std::abs(cy);
		g.p_xy = 0.5 * std::abs(cz);
		g.triangle_surface_area_sum = 0.5 * std::sqrt(cx * cx + cy * cy + cz * cz);
		g.primitive_count = 1u;
		return g;
	}

	double triangle_exact_projected_area(const triangle& t, const unit_direction& u)
	{
		if (!u.valid) return nan_value();

		const double v0x = double(t.vrts[0].x), v0y = double(t.vrts[0].y), v0z = double(t.vrts[0].z);
		const double v1x = double(t.vrts[1].x), v1y = double(t.vrts[1].y), v1z = double(t.vrts[1].z);
		const double v2x = double(t.vrts[2].x), v2y = double(t.vrts[2].y), v2z = double(t.vrts[2].z);

		const double ax = v1x - v0x, ay = v1y - v0y, az = v1z - v0z;
		const double bx = v2x - v0x, by = v2y - v0y, bz = v2z - v0z;

		const double cx = ay * bz - az * by;
		const double cy = az * bx - ax * bz;
		const double cz = ax * by - ay * bx;

		return 0.5 * std::abs(cx * u.x + cy * u.y + cz * u.z);
	}

	double triangle_exact_projected_area(const triangle& t, const vec3& direction)
	{
		return triangle_exact_projected_area(t, normalize_direction(direction));
	}

	std::vector<directional_geometry> compute_directional_geometry(const bvh2& tree, const mesh& m)
	{
		const std::vector<bvh2_node>& nodes = tree.nodes();

		std::vector<directional_geometry> out(nodes.size());
		if (nodes.empty()) return out;

		// Parents precede children, so one reverse scan is enough.
		for (i32 i = static_cast<i32>(nodes.size()) - 1; i >= 0; --i)
		{
			const bvh2_node& node = nodes[static_cast<u32>(i)];
			directional_geometry g;

			if (node.ptr.is_int)
			{
				for (u32 c = 0; c < node.ptr.child_cnt; ++c)
				{
					const u32 child = node.ptr.child_idx + c;
					DCHECK_GT(child, u32(i));
					g.add(out[child]);
				}
			}
			else
			{
				for (u32 p = 0; p < node.ptr.prim_cnt; ++p)
					g.add(triangle_directional_geometry(m.get_triangle(tree.prim_index(node.ptr.prim_idx + p))));
			}

			out[static_cast<u32>(i)] = g;
		}

		return out;
	}

	double axis_projected_area_upper(const directional_geometry& g, const unit_direction& u)
	{
		if (!u.valid) return nan_value();
		return std::abs(u.x) * g.p_yz + std::abs(u.y) * g.p_xz + std::abs(u.z) * g.p_xy;
	}

	double axis_projected_area_upper(const directional_geometry& g, const vec3& direction)
	{
		return axis_projected_area_upper(g, normalize_direction(direction));
	}

	double projected_box_area(const aabb& box, const unit_direction& u)
	{
		if (!u.valid) return nan_value();

		double ex, ey, ez;
		box_extent(box, ex, ey, ez);

		return std::abs(u.x) * ey * ez + std::abs(u.y) * ex * ez + std::abs(u.z) * ex * ey;
	}

	double projected_box_area(const aabb& box, const vec3& direction)
	{
		return projected_box_area(box, normalize_direction(direction));
	}

	directional_ratio compute_directional_ratio(const directional_geometry& g, const aabb& box,
		const unit_direction& u)
	{
		directional_ratio r;
		r.raw = nan_value();
		r.clamped = nan_value();
		r.emptiness_lower = nan_value();

		if (!u.valid) return r;

		const double b = projected_box_area(box, u);
		if (!(b > 0.0) || !finite(b)) return r;

		const double n = axis_projected_area_upper(g, u);
		if (!finite(n) || n < 0.0) return r;

		const double raw = n / b;
		if (!finite(raw)) return r;

		r.raw = raw;
		r.clamped = raw < 0.0 ? 0.0 : (raw > 1.0 ? 1.0 : raw);
		r.emptiness_lower = 1.0 - r.clamped;
		r.valid = true;
		return r;
	}

	directional_ratio compute_directional_ratio(const directional_geometry& g, const aabb& box,
		const vec3& direction)
	{
		return compute_directional_ratio(g, box, normalize_direction(direction));
	}

	const char* to_string(ratio_bin b)
	{
		switch (b)
		{
		case ratio_bin::q_00_10:  return "q_00_10";
		case ratio_bin::q_10_20:  return "q_10_20";
		case ratio_bin::q_20_30:  return "q_20_30";
		case ratio_bin::q_30_40:  return "q_30_40";
		case ratio_bin::q_40_50:  return "q_40_50";
		case ratio_bin::q_50_60:  return "q_50_60";
		case ratio_bin::q_60_70:  return "q_60_70";
		case ratio_bin::q_70_80:  return "q_70_80";
		case ratio_bin::q_80_90:  return "q_80_90";
		case ratio_bin::q_90_100: return "q_90_100";
		case ratio_bin::raw_gt_1: return "raw_gt_1";
		case ratio_bin::invalid:  return "invalid";
		default:                  return "unknown";
		}
	}

	ratio_bin classify_ratio(const directional_ratio& r)
	{
		if (!r.valid || !finite(r.raw)) return ratio_bin::invalid;
		if (r.raw > 1.0)                return ratio_bin::raw_gt_1;
		if (r.raw < 0.0)                return ratio_bin::invalid; // cannot happen: areas are non-negative

		// [0,.1) ... [.8,.9), then [.9,1.0] closed at the top.
		const u32 index = static_cast<u32>(r.raw * 10.0);
		if (index >= 9u) return ratio_bin::q_90_100;
		return static_cast<ratio_bin>(index);
	}

} // namespace bvh
