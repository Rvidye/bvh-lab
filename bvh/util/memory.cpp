#include <util/memory.h>

#include <new>
#include <vector>

#if defined(_WIN32)
#	define WIN32_LEAN_AND_MEAN
#	include <windows.h>
#	include <psapi.h>
#	pragma comment(lib, "psapi.lib")
#endif

namespace bvh
{
	u64 peak_working_set_bytes()
	{
#if defined(_WIN32)
		PROCESS_MEMORY_COUNTERS pmc{};
		pmc.cb = sizeof(pmc);
		if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
			return static_cast<u64>(pmc.PeakWorkingSetSize);
#endif
		return 0;
	}

	u64 current_working_set_bytes()
	{
#if defined(_WIN32)
		PROCESS_MEMORY_COUNTERS pmc{};
		pmc.cb = sizeof(pmc);
		if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
			return static_cast<u64>(pmc.WorkingSetSize);
#endif
		return 0;
	}

	bool probe_allocation(u64 bytes, u64& peak_after)
	{
		peak_after = peak_working_set_bytes();
		if (bytes == 0) return true;

		try
		{
			// Touch one byte per 4 KiB page so the pages are committed and the
			// working set actually reflects the request.
			std::vector<unsigned char> block(static_cast<size_t>(bytes), 0u);
			volatile unsigned char sink = 0;
			for (size_t i = 0; i < block.size(); i += 4096) sink = (block[i] ^= 1u);
			(void)sink;
			peak_after = peak_working_set_bytes();
		}
		catch (const std::bad_alloc&)
		{
			peak_after = peak_working_set_bytes();
			return false;
		}
		return true;
	}

} // namespace bvh
