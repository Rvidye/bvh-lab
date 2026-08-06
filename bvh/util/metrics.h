#pragma once

#include <bvh.h>

#include <string>
#include <utility>
#include <vector>

namespace bvh {

	class metrics
	{
		public:
			void set(const char* key, const std::string& value);
			void set(const char* key, const char* value);
			void set(const char* key, double value, int decimals = 4);
			void set(const char* key, i64 value);
			void set(const char* key, u32 value);
			void set(const char* key, int value);

			bool flush(const std::string& path) const;

			void print(FILE* out) const;

			bool empty() const { return _fields.empty(); }
			void clear() { _fields.clear(); }

		private:
			void put(const char* key, std::string value);
			std::vector<std::pair<std::string, std::string>> _fields;
	};

} // namespace bvh