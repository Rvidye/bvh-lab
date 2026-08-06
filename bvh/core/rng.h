#pragma once

#include<core/qualifiers.h>

namespace bvh
{
	// PCG32. Small, fast, device-portable, and deterministic per stream.
	struct rng
	{
		u64 state{ 0x853c49e6748fea9bULL };
		u64 inc{ 0xda3e39cb94b95bdbULL };

		rng() = default;

		BVH_DEVI explicit rng(u64 seed, u64 stream = 1u)
		{
			state = 0u;
			inc = (stream << 1u) | 1u;
			next_u32();
			state += seed;
			next_u32();
		}

		BVH_DEVI u32 next_u32()
		{
			const u64 old = state;
			state = old * 6364136223846793005ULL + inc;
			const u32 xorshifted = static_cast<u32>(((old >> 18u) ^ old) >> 27u);
			const u32 rot = static_cast<u32>(old >> 59u);
			return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31u));
		}

		// uniform in [0,1]
		BVH_DEVI f32 next_f32()
		{
			return static_cast<f32>(next_u32()  >> 8) * 0x1.0p-24f;
		}

	};
} // namespace bvh
