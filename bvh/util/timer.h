#pragma once

#include<bvh.h>
#include<chrono>

namespace bvh 
{
	// wall-clock timer
	struct timer
	{
		using clock = std::chrono::high_resolution_clock;

		clock::time_point start{ clock::now() };

		void reset() { start = clock::now(); }

		double elapsed_s() const
		{
			return std::chrono::duration<double>(clock::now() - start).count();
		}

		double elapsed_ms() const { return elapsed_s() * 1000.0; }
	};
}
