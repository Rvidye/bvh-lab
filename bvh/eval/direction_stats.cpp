#include <eval/direction_stats.h>

#include <cmath>

namespace bvh
{
	direction_weights compute_direction_weights(const rayset& training)
	{
		direction_weights w;

		double sx = 0.0, sy = 0.0, sz = 0.0;
		u64    used = 0, rejected = 0;

		const u32 n = training.size();
		for (u32 i = 0; i < n; ++i)
		{
			const double dx = double(training.d[i].x);
			const double dy = double(training.d[i].y);
			const double dz = double(training.d[i].z);

			if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) { ++rejected; continue; }

			const double len_sq = dx * dx + dy * dy + dz * dz;
			if (!(len_sq > 0.0) || !std::isfinite(len_sq)) { ++rejected; continue; }

			const double len = std::sqrt(len_sq);
			if (!(len > 0.0) || !std::isfinite(len)) { ++rejected; continue; }

			sx += std::abs(dx) / len;
			sy += std::abs(dy) / len;
			sz += std::abs(dz) / len;
			++used;
		}

		w.rays = used;
		w.rejected = rejected;

		if (used == 0) return w;

		w.wx = sx / double(used);
		w.wy = sy / double(used);
		w.wz = sz / double(used);

		// |dx|+|dy|+|dz| >= 1 for any unit vector, and each component is in [0,1].
		const double sum = w.wx + w.wy + w.wz;
		w.valid = w.wx >= 0.0 && w.wy >= 0.0 && w.wz >= 0.0
			&& std::isfinite(sum) && sum >= 0.999;
		return w;
	}

} // namespace bvh
