#include <build/geometry_loss.h>

#include <util/check.h>

#include <cmath>

namespace bvh
{
	const char* to_string(collapse_loss kind)
	{
		switch (kind)
		{
		case collapse_loss::none:           return "sah";
		case collapse_loss::directional:    return "sah+Ldir";
		case collapse_loss::scalar_density: return "sah+Lscalar";
		default:                            return "unknown";
		}
	}

	namespace
	{
		// Fx = ey*ez, Fy = ex*ez, Fz = ex*ey. An empty box has no extent.
		void face_areas(const aabb& box, double& fx, double& fy, double& fz)
		{
			const vec3   e = box.extent();
			const double ex = double(e.x), ey = double(e.y), ez = double(e.z);
			fx = ey * ez;
			fy = ex * ez;
			fz = ex * ey;
		}

		double positive(double v) { return v > 0.0 ? v : 0.0; }
	}

	f32 directional_loss(const aabb& box, const directional_geometry& g)
	{
		double fx, fy, fz;
		face_areas(box, fx, fy, fz);

		// One-sided: projected triangle areas overlap, so an axis whose projected
		// geometry already exceeds the box face contributes no emptiness.
		const double ex = positive(fx - g.p_yz);
		const double ey = positive(fy - g.p_xz);
		const double ez = positive(fz - g.p_xy);

		const double l = 2.0 * (ex + ey + ez);
		return static_cast<f32>(std::isfinite(l) ? l : 0.0);
	}

	f32 scalar_density_loss(const aabb& box, const directional_geometry& g)
	{
		const double sa = double(box.surface_area());
		const double l = positive(sa - 2.0 * g.triangle_surface_area_sum);
		return static_cast<f32>(std::isfinite(l) ? l : 0.0);
	}

	geometry_loss_terms compute_geometry_loss(const bvh2& tree, const mesh& m)
	{
		// G(n) and Atri(n), accumulated bottom-up over each subtree's triangles by
		// the already-tested descriptor. No ray information is involved.
		const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);

		const std::vector<bvh2_node>& nodes = tree.nodes();
		CHECK_EQ(g.size(), nodes.size());

		geometry_loss_terms out;
		out.ldir.resize(nodes.size());
		out.lscalar.resize(nodes.size());
		out.sa.resize(nodes.size());

		for (size_t i = 0; i < nodes.size(); ++i)
		{
			out.sa[i] = nodes[i].bounds.surface_area();
			out.ldir[i] = directional_loss(nodes[i].bounds, g[i]);
			out.lscalar[i] = scalar_density_loss(nodes[i].bounds, g[i]);
		}

		return out;
	}

	std::vector<f32> compute_internal_cost_area(const bvh2& tree, const mesh& m,
		const geometry_loss_args& args)
	{
		const std::vector<bvh2_node>& nodes = tree.nodes();

		std::vector<f32> out(nodes.size(), 0.0f);

		if (args.kind == collapse_loss::none || args.mu == 0.0)
		{
			// Bit-identical to the ordinary collapse by construction.
			for (size_t i = 0; i < nodes.size(); ++i)
				out[i] = nodes[i].bounds.surface_area();
			return out;
		}

		const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);
		CHECK_EQ(g.size(), nodes.size());

		for (size_t i = 0; i < nodes.size(); ++i)
		{
			const aabb& box = nodes[i].bounds;

			const double sa = double(box.surface_area());
			const double loss = (args.kind == collapse_loss::directional)
				? double(directional_loss(box, g[i]))
				: double(scalar_density_loss(box, g[i]));

			const double v = sa + args.mu * loss;
			out[i] = static_cast<f32>(v >= 0.0 && std::isfinite(v) ? v : sa);
		}

		return out;
	}

	loss_totals sum_internal_loss(const bvh2& tree, const mesh& m)
	{
		loss_totals t;

		const std::vector<bvh2_node>& nodes = tree.nodes();
		if (nodes.empty()) return t;

		const double root_sa = double(nodes[0].bounds.surface_area());
		if (!(root_sa > 0.0)) return t;

		const std::vector<directional_geometry> g = compute_directional_geometry(tree, m);

		for (size_t i = 0; i < nodes.size(); ++i)
		{
			if (!nodes[i].ptr.is_int) continue;
			++t.internal_nodes;
			t.directional += double(directional_loss(nodes[i].bounds, g[i])) / root_sa;
			t.scalar_density += double(scalar_density_loss(nodes[i].bounds, g[i])) / root_sa;
		}

		return t;
	}

} // namespace bvh
