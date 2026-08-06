#pragma once

#include<bvh.h>

namespace bvh 
{
	enum class log_level { verbose = 0, info = 1, warn = 2, error = 3, fatal = 4, off = 5 };

	void log_init(log_level level, const char* file = nullptr);
	void log_shutdown();

	log_level log_get_level();

	void log_message(log_level level, const char* file, int line, const char* fmt, ...);

	[[noreturn]] void log_fatal_message(const char* file, int line, const char* fmt, ...);

#define LOG_VERBOSE(...)                                                        \
	((bvh::log_level::verbose >= bvh::log_get_level())                          \
	     ? bvh::log_message(bvh::log_level::verbose, __FILE__, __LINE__, __VA_ARGS__) \
	     : (void)0)

#define LOG_INFO(...)                                                           \
	((bvh::log_level::info >= bvh::log_get_level())                             \
	     ? bvh::log_message(bvh::log_level::info, __FILE__, __LINE__, __VA_ARGS__) \
	     : (void)0)

#define LOG_WARN(...)                                                           \
	((bvh::log_level::warn >= bvh::log_get_level())                             \
	     ? bvh::log_message(bvh::log_level::warn, __FILE__, __LINE__, __VA_ARGS__) \
	     : (void)0)

#define LOG_ERROR(...)                                                          \
	((bvh::log_level::error >= bvh::log_get_level())                            \
	     ? bvh::log_message(bvh::log_level::error, __FILE__, __LINE__, __VA_ARGS__) \
	     : (void)0)

#define LOG_FATAL(...) bvh::log_fatal_message(__FILE__, __LINE__, __VA_ARGS__)
} // namespace bvh
