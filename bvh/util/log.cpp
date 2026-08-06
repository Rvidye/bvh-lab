#include <util/log.h>

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>

namespace bvh
{
	namespace
	{
		log_level g_level = log_level::info;
		FILE* g_file = nullptr;
		std::mutex g_mutex;

		const char* level_tag(log_level l)
		{
			switch (l)
			{
				case bvh::log_level::verbose: return "VERB";
				case bvh::log_level::info: return "INFO";
				case bvh::log_level::warn: return "WARN";
				case bvh::log_level::error: return "ERR";
				case bvh::log_level::fatal: return "FATAL";
				case bvh::log_level::off:
				default: return "????";
			}
		}

		// strip directory so log lines stay readable
		const char* short_file(const char* path)
		{
			const char* slash = strrchr(path, '\\');
			if (!slash) slash = strrchr(path, '/');
			return slash ? slash + 1 : path;
		}

		void emit(log_level level, const char* file, int line, const char* msg)
		{
			std::lock_guard<std::mutex> lock(g_mutex);

			FILE* out = (level >= log_level::warn) ? stderr : stdout;
			if (level >= log_level::warn)
				fprintf(out, "[%s] %s:%d: %s\n", level_tag(level), short_file(file), line, msg);
			else
				fprintf(out, "[%s] %s\n", level_tag(level), msg);
			fflush(out);

			if (g_file)
			{
				fprintf(g_file, "[%s] %s:%d: %s\n", level_tag(level), short_file(file), line, msg);
				fflush(g_file);
			}
		}
	} // namespace

	void log_init(log_level level, const char* file)
	{
		g_level = level;
		if (file && *file)
		{
			fopen_s(&g_file, file, "w");
			if (!g_file) fprintf(stderr, "[WARN] could not open log file '%s'\n", file);
		}
	}

	void log_shutdown()
	{
		if (g_file) { fclose(g_file); g_file = nullptr; }
	}

	log_level log_get_level() { return g_level; }

	void log_message(log_level level, const char* file, int line, const char* fmt, ...)
	{
		char buf[2048];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);
		emit(level, file, line, buf);
	}

	void log_fatal_message(const char* file, int line, const char* fmt, ...)
	{
		char buf[2048];
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, sizeof(buf), fmt, args);
		va_end(args);
		emit(log_level::fatal, file, line, buf);
		log_shutdown();
		abort();
	}
}// namespace bvh

