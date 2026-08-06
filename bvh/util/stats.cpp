#include <util/stats.h>

#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace bvh
{
	namespace 
	{
		// Registered callbacks
		std::vector<stat_registerer::accum_fn>& callbacks()
		{
			static std::vector<stat_registerer::accum_fn> v;
			return v;
		}

		struct ratio_entry { i64 num{ 0 }, denom{ 0 }; };
		struct dist_entry { i64 sum{ 0 }, count{ 0 }, lo{ INT64_MAX }, hi{ INT64_MIN }; };

		// Process-wide totals.
		struct global_state
		{
			std::mutex                     mutex;
			std::map<std::string, i64>         counters;
			std::map<std::string, ratio_entry> ratios;
			std::map<std::string, ratio_entry> percents;
			std::map<std::string, dist_entry>  distributions;
		};

		global_state& state()
		{
			static global_state s;
			return s;
		}

		// Split "Category/metric" for grouped printing.
		void split_name(const std::string& full, std::string& category, std::string& metric)
		{
			const size_t slash = full.find('/');
			if (slash == std::string::npos) { category = "General"; metric = full; }
			else { category = full.substr(0, slash); metric = full.substr(slash + 1); }
		}

		std::string with_commas(i64 v)
		{
			std::string s = std::to_string(v);
			const bool neg = !s.empty() && s[0] == '-';
			const size_t start = neg ? 1u : 0u;
			for (size_t i = s.size(); i > start + 3; )
			{
				i -= 3;
				s.insert(i, ",");
			}
			return s;
		}
	} // namespace

	stat_registerer::stat_registerer(accum_fn fn)
	{
		callbacks().push_back(fn);
	}

	void stat_registerer::call_callbacks(stats_accumulator& accum)
	{
		for (accum_fn fn : callbacks()) fn(accum);
	}

	void stats_accumulator::report_counter(const char* name, i64 value)
	{
		if (value == 0) return;
		state().counters[name] += value;
	}

	void stats_accumulator::report_ratio(const char* name, i64 num, i64 denom)
	{
		if (denom == 0) return;
		auto& e = state().ratios[name];
		e.num += num;
		e.denom += denom;
	}

	void stats_accumulator::report_percent(const char* name, i64 num, i64 denom)
	{
		if (denom == 0) return;
		auto& e = state().percents[name];
		e.num += num;
		e.denom += denom;
	}

	void stats_accumulator::report_distribution(const char* name, i64 sum, i64 count, i64 lo, i64 hi)
	{
		if (count == 0) return;
		auto& e = state().distributions[name];
		e.sum += sum;
		e.count += count;
		e.lo = std::min(e.lo, lo);
		e.hi = std::max(e.hi, hi);
	}

	void stats_accumulator::clear()
	{
		global_state& s = state();
		s.counters.clear();
		s.ratios.clear();
		s.percents.clear();
		s.distributions.clear();
	}

	void stats_accumulator::print(FILE* out) const
	{
		global_state& s = state();

		fprintf(out, "\nStatistics:\n");

		std::string last_category, category, metric;

		auto header = [&](const std::string& full) {
			split_name(full, category, metric);
			if (category != last_category)
			{
				fprintf(out, "  %s\n", category.c_str());
				last_category = category;
			}
			};

		for (const auto& [name, value] : s.counters)
		{
			header(name);
			fprintf(out, "    %-44s %14s\n", metric.c_str(), with_commas(value).c_str());
		}

		for (const auto& [name, e] : s.ratios)
		{
			header(name);
			fprintf(out, "    %-44s %14s / %s = %.3f\n", metric.c_str(),
				with_commas(e.num).c_str(), with_commas(e.denom).c_str(),
				double(e.num) / double(e.denom));
		}

		for (const auto& [name, e] : s.percents)
		{
			header(name);
			fprintf(out, "    %-44s %14s / %s = %.2f%%\n", metric.c_str(),
				with_commas(e.num).c_str(), with_commas(e.denom).c_str(),
				100.0 * double(e.num) / double(e.denom));
		}

		for (const auto& [name, e] : s.distributions)
		{
			header(name);
			fprintf(out, "    %-44s %.3f avg [%s .. %s] n=%s\n", metric.c_str(),
				double(e.sum) / double(e.count),
				with_commas(e.lo).c_str(), with_commas(e.hi).c_str(),
				with_commas(e.count).c_str());
		}
	}

	void report_thread_stats()
	{
		static stats_accumulator accum;
		std::lock_guard<std::mutex> lock(state().mutex);
		stat_registerer::call_callbacks(accum);
	}

	void print_stats(FILE* out)
	{
		static stats_accumulator accum;
		accum.print(out);
	}

	void clear_stats()
	{
		static stats_accumulator accum;
		accum.clear();
	}

} // namespace bvh
