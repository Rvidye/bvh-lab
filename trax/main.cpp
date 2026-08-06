#include <core/trace_stats.h>
#include <reference/brute_force.h>
#include <util/camera.h>
#include <util/image.h>
#include <util/log.h>
#include <util/mesh.h>
#include <util/metrics.h>
#include <util/parallel.h>
#include <util/stats.h>
#include <util/timer.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace bvh;

namespace 
{
	class args
	{
	public :
		args(int argc, char** argv)
		{
			for (int i = 1; i < argc; ++i)
			{
				std::string a = argv[i];
				if (a.rfind("--", 0) != 0)
				{
					LOG_WARN("ignoring positional argument '%s'", a.c_str());
					continue;
				}
				a = a.substr(2);
				const size_t eq = a.find("=");
				if (eq == std::string::npos) _values[a] = "1";
				else						 _values[a.substr(0, eq)] = a.substr(eq + 1);
			}
		}

		std::string get(const char* key, const char* fallback) const
		{
			auto it = _values.find(key);
			return it == _values.end() ? std::string(fallback) : it->second;
		}

		u32 get_u32(const char* key, u32 fallback) const
		{
			auto it = _values.find(key);
			return it == _values.end() ? fallback : u32(std::strtoul(it->second.c_str(), nullptr, 10));
		}

		f32 get_f32(const char* key, f32 fallback) const
		{
			auto it = _values.find(key);
			return it == _values.find(key) ? fallback : f32(std::atof(it->second.c_str()));
		}

		bool has(const char* key) const { return _values.count(key) != 0; }

	private:
		std::map<std::string, std::string> _values;
	};

	void ensure_parent_dir(const std::string& path)
	{
		const std::filesystem::path p(path);
		if (p.has_parent_path() && !p.parent_path().empty())
			std::filesystem::create_directories(p.parent_path());
	}

	void print_usage()
	{
		printf(
			"trax CPU tracer\n"
			"\n"
			"  --scene=<path.obj>     mesh to load        (default scenes/bunny.obj)\n"
			"  --scale=<f>            uniform mesh scale  (default 1.0)\n"
			"  --width=<n>            image width         (default 512)\n"
			"  --height=<n>           image height        (default 512)\n"
			"  --out=<path.png>       image output        (default results/d1_normals.png)\n"
			"  --csv=<path.csv>       metrics row output  (default results/d1.csv)\n"
			"  --threads=<n>          0 = hardware        (default 0)\n"
			"  --verbose              verbose logging\n"
			"  --help\n");
	}
} // namespace

int main(int argc, char** argv)
{
	const args opts(argc, argv);

	if (opts.has("help")) { print_usage(); return 0; }

	log_init(opts.has("verbose") ? log_level::verbose : log_level::info);

	const std::string scene_path = opts.get("scene", "scenes/teapot.obj");
	const std::string out_path = opts.get("out", "results/out.png");
	const std::string csv_path = opts.get("csv", "results/out.csv");
	const u32 width = opts.get_u32("width", 512);
	const u32 height = opts.get_u32("height", 512);
	const u32 threads = opts.get_u32("threads", 0);
	const f32 scale = opts.get_f32("scale", 1.0f);

	mesh m;
	if (!m.load_obj(scene_path, scale))
	{
		LOG_ERROR("could not load scene '%s'", scene_path.c_str());
		return 1;
	}

	const aabb& b = m.bounds();
	LOG_INFO("bounds: (%.3f %.3f %.3f) .. (%.3f %.3f %.3f)", b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z);

	const camera cam = camera::frame_bounds(b, width, height);

	image img(width, height);

	LOG_INFO("rendering %ux%u with %u threads, brute force over %u triangles...", width, height, threads ? threads : hardware_threads(), m.triangle_count());

	const render_result r = render_normals(m, cam, img, threads);

	ensure_parent_dir(out_path);
	if (!img.write_png(out_path)) return 1;

	const double prim_tests = double(r.rays) * double(m.triangle_count());
	metrics row;
	row.set("scene", std::filesystem::path(scene_path).filename().string());
	row.set("triangles", m.triangle_count());
	row.set("builder", "none");
	row.set("layout", "none");
	row.set("traversal", "brute_force");
	row.set("width", width);
	row.set("height", height);
	row.set("rays", i64(r.rays));
	row.set("hits", i64(r.hits));
	row.set("hit_rate", r.rays ? double(r.hits) / double(r.rays) : 0.0);
	row.set("threads", threads ? threads : hardware_threads());
	row.set("trace_s", r.seconds);
	row.set("mrays_s", r.mrays_per_second());
	row.set("prim_tests", prim_tests, 0);
	row.set("prim_tests_per_ray", r.rays ? prim_tests / double(r.rays) : 0.0, 1);

	ensure_parent_dir(csv_path);
	row.flush(csv_path);
	row.print(stdout);

	report_thread_stats();
	print_stats(stdout);

	LOG_INFO("done in %.2f s (%.3f MRays/s)", r.seconds, r.mrays_per_second());
	log_shutdown();
	return 0;
}

