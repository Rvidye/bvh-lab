#pragma once

// host/device qualifiers
// Rules:
// POD only. No virtuals, no exceptions, no STL.
// core/ never allocates and never owns. It takes const T* plus counts.
// all cross-references are uint32 indices into flat arrays, never pointers.

#ifdef __CUDACC__
#define BVH_DEV __host__ __device__
#else
#define BVH_DEV 
#endif // __CUDACC__

#define BVH_INLINE inline
#define BVH_DEVI BVH_DEV BVH_INLINE

namespace bvh
{
	using u8 = unsigned char;
	using u16 = unsigned short;
	using u32 = unsigned int;
	using u64 = unsigned long long;
	using i32 = int;
	using i64 = long long;
	using f32 = float;

	constexpr u32 invalid_id = ~0u;
	constexpr f32 f32_max = 1e32f;
} // namespace bvh
