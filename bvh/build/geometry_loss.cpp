#include <build/geometry_loss.h>

#include <util/check.h>

#include <cmath>

namespace bvh
{
	const char* to_string(collapse_loss kind)
	{
		switch (kind)
		{
		case collapse_loss::none:                return "sah";
		case collapse_loss::directional:         return "sah+Ldir";
		case collapse_loss::scalar_density:      return "sah+Lscalar";
		case collapse_loss::directional_min:     return "sah+Ldir_min";
		case collapse_loss::directional_softmin: return "sah+Ldir_softmin";
		case collapse_loss::directional_spread:  return "sah+Ldir_spread";
		default:                                 return "unknown";
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

	// ------------------------------------------------- per-axis fill variants

	double saturating_fill(double q)
	{
		if (!(q > 0.0) || !std::isfinite(q)) return 0.0;
		return 1.0 - std::exp(-q);
	}

	axis_fill compute_axis_fill(const aabb& box, const directional_geometry& g)
	{
		double f[3];
		face_areas(box, f[0], f[1], f[2]);

		const double gp[3] = { g.p_yz, g.p_xz, g.p_xy };

		axis_fill out;
		for (u32 i = 0; i < 3; ++i)
		{
			if (!(f[i] > 0.0) || !std::isfinite(f[i]))
			{
				out.valid[i] = false;
				out.q[i] = 0.0;
				out.fill[i] = 0.0;
				out.saturated[i] = false;
				continue;
			}
			const double q = gp[i] / f[i];
			out.valid[i] = std::isfinite(q);
			out.q[i] = out.valid[i] ? q : 0.0;
			out.fill[i] = out.valid[i] ? saturating_fill(q) : 0.0;
			out.saturated[i] = out.valid[i] && q >= 1.0;
			if (out.valid[i]) ++out.valid_count;
		}
		return out;
	}

	namespace
	{
		// Gathers the fills the policy says are usable. Returns false when the
		// node carries no usable axis at all, which the callers report as no loss.
		bool usable_fills(const axis_fill& a, degenerate_axis_policy policy,
			double* out, u32& count)
		{
			count = 0;
			for (u32 i = 0; i < 3; ++i)
			{
				if (a.valid[i])            out[count++] = a.fill[i];
				else if (policy == degenerate_axis_policy::treat_as_full)
					out[count++] = 1.0;
			}
			return count > 0;
		}
	}

	double directional_min_fill(const aabb& box, const directional_geometry& g,
		degenerate_axis_policy policy)
	{
		const axis_fill a = compute_axis_fill(box, g);

		double f[3];
		u32    n = 0;
		if (!usable_fills(a, policy, f, n)) return 1.0;   // no usable axis: no waste

		double lo = f[0];
		for (u32 i = 1; i < n; ++i) lo = f[i] < lo ? f[i] : lo;
		return lo;
	}

	namespace
	{
		double axis_weight(double face, double exponent)
		{
			if (exponent == 0.0) return 1.0;
			if (exponent == 1.0) return face;
			return std::pow(face, exponent);
		}

		double apply_fill_map(double q, fill_map map)
		{
			if (map == fill_map::exponential) return saturating_fill(q);
			if (!(q > 0.0) || !std::isfinite(q)) return 0.0;
			return q > 1.0 ? 1.0 : q;
		}
	}

	double axis_fill_shape(const aabb& box, const directional_geometry& g,
		const fill_shape_args& args)
	{
		double f[3];
		face_areas(box, f[0], f[1], f[2]);

		const double gp[3] = { g.p_yz, g.p_xz, g.p_xy };

		double fill[3];
		double weight[3];
		u32    n = 0;

		for (u32 i = 0; i < 3; ++i)
		{
			const bool valid = (f[i] > 0.0) && std::isfinite(f[i]);
			double v, w;

			if (valid)
			{
				const double q = gp[i] / f[i];
				if (!std::isfinite(q)) continue;
				v = apply_fill_map(q, args.map);
				w = axis_weight(f[i], args.weight_exponent);
			}
			else if (args.degenerate == degenerate_axis_policy::treat_as_full)
			{
				// Nothing to enter, so nothing is wasted. It carries no face area
				// either, so under any positive exponent it cannot contribute to a
				// weighted mean; only the unweighted case can see it.
				v = 1.0;
				w = args.weight_exponent == 0.0 ? 1.0 : 0.0;
			}
			else continue;

			if (!std::isfinite(v) || !std::isfinite(w)) continue;
			fill[n] = v;
			weight[n] = w;
			++n;
		}

		if (n == 0) return 1.0;   // no usable axis: no waste

		if (args.aggregate == fill_aggregate::min)
		{
			double lo = fill[0];
			for (u32 i = 1; i < n; ++i) lo = fill[i] < lo ? fill[i] : lo;
			return lo;
		}

		double wsum = 0.0, vsum = 0.0;
		for (u32 i = 0; i < n; ++i)
		{
			wsum += weight[i];
			vsum += weight[i] * fill[i];
		}

		if (!(wsum > 0.0))
		{
			// Every weight vanished. Fall back to the unweighted mean rather than
			// reporting a fill of 0, which would read as "entirely empty".
			double s = 0.0;
			for (u32 i = 0; i < n; ++i) s += fill[i];
			return s / double(n);
		}
		return vsum / wsum;
	}

	double directional_mean_fill(const aabb& box, const directional_geometry& g)
	{
		// Exactly 1 - Ldir/SA, so this is the CURRENT summed formulation on the
		// same [0,1] scale as min_fill.
		const double sa = double(box.surface_area());
		if (!(sa > 0.0)) return 1.0;
		const double l = double(directional_loss(box, g));
		const double fill = 1.0 - l / sa;
		return fill < 0.0 ? 0.0 : (fill > 1.0 ? 1.0 : fill);
	}

	f32 directional_min_loss(const aabb& box, const directional_geometry& g,
		degenerate_axis_policy policy)
	{
		const double sa = double(box.surface_area());
		const double l = sa * (1.0 - directional_min_fill(box, g, policy));
		return static_cast<f32>(std::isfinite(l) ? positive(l) : 0.0);
	}

	f32 directional_softmin_loss(const aabb& box, const directional_geometry& g,
		double beta, degenerate_axis_policy policy)
	{
		const axis_fill a = compute_axis_fill(box, g);

		double f[3];
		u32    n = 0;
		if (!usable_fills(a, policy, f, n)) return 0.0f;

		// Boltzmann-weighted mean: beta = 0 is the plain mean, beta -> inf the min.
		double wsum = 0.0, vsum = 0.0;
		for (u32 i = 0; i < n; ++i)
		{
			const double w = std::exp(-beta * f[i]);
			wsum += w;
			vsum += w * f[i];
		}
		const double shape = wsum > 0.0 ? vsum / wsum : f[0];

		const double sa = double(box.surface_area());
		const double l = sa * (1.0 - shape);
		return static_cast<f32>(std::isfinite(l) ? positive(l) : 0.0);
	}

	f32 directional_spread_loss(const aabb& box, const directional_geometry& g,
		degenerate_axis_policy policy)
	{
		const axis_fill a = compute_axis_fill(box, g);

		double f[3];
		u32    n = 0;
		if (!usable_fills(a, policy, f, n)) return 0.0f;

		double lo = f[0], hi = f[0];
		for (u32 i = 1; i < n; ++i)
		{
			lo = f[i] < lo ? f[i] : lo;
			hi = f[i] > hi ? f[i] : hi;
		}

		const double sa = double(box.surface_area());
		const double l = sa * (hi - lo);
		return static_cast<f32>(std::isfinite(l) ? positive(l) : 0.0);
	}

	f32 evaluate_loss(const aabb& box, const directional_geometry& g,
		const geometry_loss_args& args)
	{
		switch (args.kind)
		{
		case collapse_loss::directional:         return directional_loss(box, g);
		case collapse_loss::scalar_density:      return scalar_density_loss(box, g);
		case collapse_loss::directional_min:     return directional_min_loss(box, g, args.degenerate);
		case collapse_loss::directional_softmin: return directional_softmin_loss(box, g, args.softmin_beta, args.degenerate);
		case collapse_loss::directional_spread:  return directional_spread_loss(box, g, args.degenerate);
		default:                                 return 0.0f;
		}
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
			const double loss = double(evaluate_loss(box, g[i], args));

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
