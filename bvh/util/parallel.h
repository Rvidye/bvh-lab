#pragma once

#include <bvh.h>
#include <util/stats.h>

#include <algorithm>
#include <atomic>
#include <thread>
#include <vector>

namespace bvh
{
	inline u32 hardware_threads()
	{
		const u32 n = std::thread::hardware_concurrency();
		return n ? n : 1u;
	}

	// Dynamic-chunk parallel for over [0, count).
	// Each worker calls report_thread_stats() before exiting, so STAT_COUNTER values accumulated in fn are not lost.

	template <typename F>
	void parallel_for(u32 count, F&& fn, u32 chunk = 0, u32 thread_count = 0)
	{
		if (count == 0) return;

		if (thread_count == 0) thread_count = hardware_threads();
		thread_count = std::min(thread_count, count);

		if (chunk == 0)
		{
			// Aim for ~64 chunks per thread: small enough to balance, large enough
			// that the atomic is not the bottleneck.
			chunk = std::max(1u, count / (thread_count * 64u));
		}

		if (thread_count == 1)
		{
			for (u32 i = 0; i < count; ++i) fn(i);
			report_thread_stats();
			return;
		}

		std::atomic<u32> cursor{ 0 };

		auto worker = [&]() {
			for (;;)
			{
				const u32 begin = cursor.fetch_add(chunk, std::memory_order_relaxed);
				if (begin >= count) break;
				const u32 end = std::min(begin + chunk, count);
				for (u32 i = begin; i < end; ++i) fn(i);
			}
			report_thread_stats();
			};

		std::vector<std::thread> threads;
		threads.reserve(thread_count - 1);
		for (u32 t = 0; t + 1 < thread_count; ++t) threads.emplace_back(worker);
		worker(); // the calling thread pulls its weight too
		for (std::thread& t : threads) t.join();
	}
}// namespace bvh