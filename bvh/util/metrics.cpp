#include <util/metrics.h>

#include <util/log.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace bvh {

	namespace {

		// Minimal CSV qutoing
		std::string csv_escape(const std::string& s)
		{
			const bool needs = s.find_first_of(",\"\n\r") != std::string::npos;
			if (!needs) return s;

			std::string out = "\"";
			for (char c : s)
			{
				if (c == '"') out += '"';
				out += c;
			}
			out += '"';
			return out;
		}

		std::string read_first_line(const std::string& path)
		{
			std::ifstream in(path);
			if (!in.is_open()) return {};
			std::string line;
			std::getline(in, line);
			if (!line.empty() && line.back() == '\r') line.pop_back();
			return line;
		}

	} // namespace

	void metrics::put(const char* key, std::string value)
	{
		for (auto& [k, v] : _fields)
		{
			if (k == key) { v = std::move(value); return; }
		}
		_fields.emplace_back(key, std::move(value));
	}

	void metrics::set(const char* key, const std::string& value) { put(key, value); }
	void metrics::set(const char* key, const char* value) { put(key, value); }
	void metrics::set(const char* key, i64 value) { put(key, std::to_string(value)); }
	void metrics::set(const char* key, u32 value) { put(key, std::to_string(value)); }
	void metrics::set(const char* key, int value) { put(key, std::to_string(value)); }

	void metrics::set(const char* key, double value, int decimals)
	{
		char buf[64];
		snprintf(buf, sizeof(buf), "%.*f", decimals, value);
		put(key, buf);
	}

	bool metrics::flush(const std::string& path) const
	{
		if (_fields.empty()) return true;

		std::ostringstream header, row;
		for (size_t i = 0; i < _fields.size(); ++i)
		{
			if (i) { header << ','; row << ','; }
			header << csv_escape(_fields[i].first);
			row << csv_escape(_fields[i].second);
		}

		const std::string existing = read_first_line(path);
		const bool is_new = existing.empty();

		if (!is_new && existing != header.str())
		{
			LOG_ERROR("metrics: header mismatch for '%s'; refusing to append.\n"
				"  file:  %s\n"
				"  row:   %s",
				path.c_str(), existing.c_str(), header.str().c_str());
			return false;
		}

		std::ofstream out(path, std::ios::app);
		if (!out.is_open())
		{
			LOG_ERROR("metrics: could not open '%s' for append", path.c_str());
			return false;
		}

		if (is_new) out << header.str() << '\n';
		out << row.str() << '\n';
		return true;
	}

	void metrics::print(FILE* out) const
	{
		size_t width = 0;
		for (const auto& [k, v] : _fields) width = std::max(width, k.size());

		fprintf(out, "\nResults:\n");
		for (const auto& [k, v] : _fields)
			fprintf(out, "  %-*s  %s\n", int(width), k.c_str(), v.c_str());
	}

} // namespace bvh