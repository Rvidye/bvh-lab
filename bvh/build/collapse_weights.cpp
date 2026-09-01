#include <build/collapse_weights.h>

#include <util/check.h>

#include <cmath>

namespace bvh
{
	std::vector<f32> compute_collapse_weights(const bvh2& tree, double lambda,
		const direction_weights& directions)
	{
		const std::vector<bvh2_node>& nodes = tree.nodes();
		std::vector<f32> w(nodes.size(), 0.0f);

		if (lambda == 0.0)
		{
			// Bit-identical to the ordinary collapse by construction.
			for (size_t i = 0; i < nodes.size(); ++i)
				w[i] = nodes[i].bounds.surface_area();
			return w;
		}

		CHECK(directions.valid);

		const double wx = directions.wx;
		const double wy = directions.wy;
		const double wz = directions.wz;

		for (size_t i = 0; i < nodes.size(); ++i)
		{
			const aabb& b = nodes[i].bounds;

			const vec3   e = b.extent();          // zero for an empty box
			const double ex = double(e.x), ey = double(e.y), ez = double(e.z);

			const double fx = ey * ez;            // YZ face
			const double fy = ex * ez;            // XZ face
			const double fz = ex * ey;            // XY face

			const double adir = wx * fx + wy * fy + wz * fz;
			const double sa = double(b.surface_area());

			const double blended = (1.0 - lambda) * sa + lambda * 4.0 * adir;
			w[i] = static_cast<f32>(blended >= 0.0 && std::isfinite(blended) ? blended : 0.0);
		}

		return w;
	}

} // namespace bvh
