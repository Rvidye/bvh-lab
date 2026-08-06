#pragma once

#include <bvh.h>
#include <cstdint>
#include <cstdio>

// self registering thread-local counters

namespace bvh 
{
	class stats_accumulator;

	class stat_registerer
	{
		public:
			using accum_fn = void (*)(stats_accumulator&);
			explicit stat_registerer(accum_fn fn);
			static void call_callbacks(stats_accumulator& accum);
	};

	class stats_accumulator
	{
		public:
			void report_counter(const char* name, i64 value);
			void report_ratio(const char* name, i64 num, i64 denom);
			void report_percent(const char* name, i64 num, i64 denom);
			void report_distribution(const char* name, i64 sum, i64 count, i64 lo, i64 hi);

			void print(FILE* out) const;
			void clear();
	};

	// Merge the calling thread's counters into the process-wide accumulator and zero them. Safe to call from many threads.
	void report_thread_stats();

	void print_stats(FILE* out);
	void clear_stats();

	struct stat_distribution
	{
		i64 sum{ 0 };
		i64 count{ 0 };
		i64 lo{ INT64_MAX };
		i64 hi{ INT64_MIN };

		void operator<<(i64 v)
		{
			sum += v;
			count += 1;
			if (v < lo) lo = v;
			if (v > hi) hi = v;
		}
	};

	#define STAT_COUNTER(title, var)                                               \
		static thread_local bvh::i64 var;                                          \
		static bvh::stat_registerer bvh_stats_reg_##var(                           \
			[](bvh::stats_accumulator& accum) {                                    \
				accum.report_counter(title, var);                                  \
				var = 0;                                                           \
			})

	#define STAT_RATIO(title, num_var, denom_var)                                  \
		static thread_local bvh::i64 num_var, denom_var;                           \
		static bvh::stat_registerer bvh_stats_reg_##num_var(                       \
			[](bvh::stats_accumulator& accum) {                                    \
				accum.report_ratio(title, num_var, denom_var);                     \
				num_var = 0;                                                       \
				denom_var = 0;                                                     \
			})

	#define STAT_PERCENT(title, num_var, denom_var)                                \
		static thread_local bvh::i64 num_var, denom_var;                           \
		static bvh::stat_registerer bvh_stats_reg_##num_var(                       \
			[](bvh::stats_accumulator& accum) {                                    \
				accum.report_percent(title, num_var, denom_var);                   \
				num_var = 0;                                                       \
				denom_var = 0;                                                     \
			})

	#define STAT_DISTRIBUTION(title, var)                                          \
		static thread_local bvh::stat_distribution var;                            \
		static bvh::stat_registerer bvh_stats_reg_##var(                           \
			[](bvh::stats_accumulator& accum) {                                    \
				accum.report_distribution(title, var.sum, var.count, var.lo, var.hi); \
				var = bvh::stat_distribution{};                                    \
			})



} //namespace bvh


