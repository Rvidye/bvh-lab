#pragma once

#include <bvh.h>

namespace bvh
{
	// Process memory, for feasibility measurement rather than for any hot path.
	// Returns 0 when the platform does not provide the figure.

	u64 peak_working_set_bytes();
	u64 current_working_set_bytes();

	// Attempts a single contiguous allocation of the requested size and touches
	// one byte per page so the reservation is actually committed. Returns false
	// if the allocation failed. Used to answer "would this have fit?" empirically
	// instead of by arithmetic alone.
	bool probe_allocation(u64 bytes, u64& peak_after);

} // namespace bvh
