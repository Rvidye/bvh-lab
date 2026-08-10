#include <eval/rayset.h>

#include <core/rng.h>
#include <core/sampling.h>
#include <core/traverse_bvh2.h>
#include <eval/trace.h>
#include <util/check.h>
#include <util/log.h>
#include <util/timer.h>

#include <cstring>
#include <filesystem>
#include <fstream>

namespace bvh {

	const char* to_string(ray_distribution d)
	{
		switch (d)
		{
		case ray_distribution::primary:    return "primary";
		case ray_distribution::shadow_ao:  return "shadow_ao";
		case ray_distribution::reflection: return "reflection";
		case ray_distribution::diffuse_1:  return "diffuse_1";
		case ray_distribution::diffuse_n:  return "diffuse_n";
		case ray_distribution::incoherent: return "incoherent";
		default:                           return "unknown";
		}
	}

	bool ray_distribution_from_string(const std::string& s, ray_distribution& out)
	{
		for (ray_distribution d : all_ray_distributions)
		{
			if (s == to_string(d)) { out = d; return true; }
		}
		return false;
	}

	void rayset::clear()
	{
		o.clear();
		d.clear();
		t_min.clear();
		t_max.clear();
	}

	void rayset::reserve(u32 n)
	{
		o.reserve(n);
		d.reserve(n);
		t_min.reserve(n);
		t_max.reserve(n);
	}

	void rayset::push(const ray& r)
	{
		o.push_back(r.o);
		d.push_back(r.d);
		t_min.push_back(r.t_min);
		t_max.push_back(r.t_max);
	}

	namespace {

		// Offset a secondary ray origin off the surface it came from.
		BVH_INLINE vec3 offset_origin(const vec3& p, const vec3& n, f32 t)
		{
			const f32 eps = bvh::max(1e-4f, t * 1e-4f);
			return p + n * eps;
		}

		struct surface_hit
		{
			vec3 p;
			vec3 n;   // geometric normal, flipped to face the incoming ray
			f32  t;
			bool valid{ false };
		};

		// Traces one ray and returns the shading frame at the hit
		surface_hit trace_surface(const bvh2_view& view, const mesh& m, const bvh2& tree,
			const mesh_prims<robust_mode>& prims, const ray& r)
		{
			surface_hit s;

			hit h;
			h.t = r.t_max;
			null_stats stats;
			if (!intersect<robust_mode>(view, r, h, prims, stats)) return s;

			const u32 prim = tree.prim_index(h.id);

			s.t = h.t;
			s.p = r.o + r.d * h.t;
			s.n = m.geometric_normal(prim);
			if (dot(s.n, r.d) > 0.0f) s.n = -s.n;
			s.valid = true;
			return s;
		}

		constexpr u32 rayset_magic = 0x53595242u; // "BRYS"
		constexpr u32 rayset_version = 1u;

		struct file_header
		{
			u32 magic;
			u32 version;
			u32 distribution;
			u32 count;
			u64 seed;
			u32 scene_len;
			u32 _pad;
		};

	} // namespace

	std::string rayset_filename(const std::string& scene, ray_distribution d, const rayset_args& a)
	{
		char buf[512];
		snprintf(buf, sizeof(buf), "%s_%s_%ux%u_b%u_s%llu.rays",
			scene.c_str(), to_string(d), a.width, a.height, a.bounces,
			static_cast<unsigned long long>(a.seed));
		return buf;
	}

	rayset rayset::generate(ray_distribution dist, const mesh& m, const bvh2& tree,
		const camera& cam, const rayset_args& args)
	{
		CHECK(!m.empty());
		CHECK(!tree.empty());

		timer timer_total;

		rayset out;
		out.distribution = dist;
		out.seed = args.seed;

		const bvh2_view view = tree.view();
		const auto      prims = make_prims<robust_mode>(m, tree);

		const f32 diagonal = length(m.bounds().extent());
		const f32 ao_length = bvh::max(diagonal * args.ao_radius_fraction, 1e-4f);

		const u32 pixels = cam.width() * cam.height();

		if (dist == ray_distribution::incoherent)
		{
			out.reserve(args.incoherent_count);

			const vec3 lo = m.bounds().min;
			const vec3 ex = m.bounds().extent();

			for (u32 i = 0; i < args.incoherent_count; ++i)
			{
				rng random(args.seed, i + 1u);
				const vec3 p(lo.x + ex.x * random.next_f32(),
					lo.y + ex.y * random.next_f32(),
					lo.z + ex.z * random.next_f32());
				const vec3 dir = uniform_sphere(random.next_f32(), random.next_f32());
				out.push(ray(p, dir));
			}
		}
		else if (dist == ray_distribution::primary)
		{
			out.reserve(pixels);
			for (u32 j = 0; j < cam.height(); ++j)
				for (u32 i = 0; i < cam.width(); ++i)
					out.push(cam.generate_ray_through_pixel(i, j));
		}
		else
		{
			// Every remaining distribution starts from a primary hit.
			out.reserve(pixels);

			for (u32 j = 0; j < cam.height(); ++j)
			{
				for (u32 i = 0; i < cam.width(); ++i)
				{
					const u32 pixel = j * cam.width() + i;
					rng random(args.seed, pixel + 1u);

					ray         current = cam.generate_ray_through_pixel(i, j);
					surface_hit s = trace_surface(view, m, tree, prims, current);
					if (!s.valid) continue; // a miss spawns nothing

					if (dist == ray_distribution::shadow_ao)
					{
						const vec3 dir = cosine_hemisphere(s.n, random.next_f32(), random.next_f32());
						ray r(offset_origin(s.p, s.n, s.t), dir);
						r.t_max = ao_length; // bounded: this is what makes AO cheap
						out.push(r);
					}
					else if (dist == ray_distribution::reflection)
					{
						const vec3 dir = current.d - s.n * (2.0f * dot(current.d, s.n));
						out.push(ray(offset_origin(s.p, s.n, s.t), dir));
					}
					else if (dist == ray_distribution::diffuse_1)
					{
						const vec3 dir = cosine_hemisphere(s.n, random.next_f32(), random.next_f32());
						out.push(ray(offset_origin(s.p, s.n, s.t), dir));
					}
					else // diffuse_n
					{
						bool alive = true;
						for (u32 b = 0; b + 1 < args.bounces && alive; ++b)
						{
							const vec3 dir = cosine_hemisphere(s.n, random.next_f32(), random.next_f32());
							current = ray(offset_origin(s.p, s.n, s.t), dir);

							s = trace_surface(view, m, tree, prims, current);
							alive = s.valid;
						}
						if (!alive) continue; // escaped before reaching the target depth

						const vec3 dir = cosine_hemisphere(s.n, random.next_f32(), random.next_f32());
						out.push(ray(offset_origin(s.p, s.n, s.t), dir));
					}
				}
			}
		}

		LOG_INFO("rayset[%s]: %u rays (%.0f ms)", to_string(dist), out.size(), timer_total.elapsed_ms());
		return out;
	}

	u64 rayset::hash() const
	{
		// FNV-1a over raw bits.
		u64 h = 14695981039346656037ull;
		auto mix = [&h](const void* p, size_t n) {
			const u8* b = static_cast<const u8*>(p);
			for (size_t i = 0; i < n; ++i)
			{
				h ^= b[i];
				h *= 1099511628211ull;
			}
			};

		const u32 tag = static_cast<u32>(distribution);
		mix(&tag, sizeof(tag));

		const u32 n = size();
		mix(&n, sizeof(n));

		for (u32 i = 0; i < n; ++i)
		{
			mix(&o[i], sizeof(vec3));
			mix(&d[i], sizeof(vec3));
			mix(&t_min[i], sizeof(f32));
			mix(&t_max[i], sizeof(f32));
		}
		return h;
	}

	bool rayset::save(const std::string& path) const
	{
		const std::filesystem::path p(path);
		if (p.has_parent_path() && !p.parent_path().empty())
			std::filesystem::create_directories(p.parent_path());

		std::ofstream f(path, std::ios::binary);
		if (!f.is_open())
		{
			LOG_ERROR("rayset: cannot open '%s' for write", path.c_str());
			return false;
		}

		file_header hdr{};
		hdr.magic = rayset_magic;
		hdr.version = rayset_version;
		hdr.distribution = static_cast<u32>(distribution);
		hdr.count = size();
		hdr.seed = seed;
		hdr.scene_len = static_cast<u32>(scene.size());

		f.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
		f.write(scene.data(), hdr.scene_len);

		const u32 n = hdr.count;
		f.write(reinterpret_cast<const char*>(o.data()), sizeof(vec3) * n);
		f.write(reinterpret_cast<const char*>(d.data()), sizeof(vec3) * n);
		f.write(reinterpret_cast<const char*>(t_min.data()), sizeof(f32) * n);
		f.write(reinterpret_cast<const char*>(t_max.data()), sizeof(f32) * n);

		if (!f.good())
		{
			LOG_ERROR("rayset: write failed for '%s'", path.c_str());
			return false;
		}
		return true;
	}

	bool rayset::load(const std::string& path)
	{
		std::ifstream f(path, std::ios::binary);
		if (!f.is_open()) return false;

		file_header hdr{};
		f.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
		if (!f.good() || hdr.magic != rayset_magic || hdr.version != rayset_version)
		{
			return false;
		}

		scene.assign(hdr.scene_len, '\0');
		if (hdr.scene_len) f.read(scene.data(), hdr.scene_len);

		distribution = static_cast<ray_distribution>(hdr.distribution);
		seed = hdr.seed;

		const u32 n = hdr.count;
		o.resize(n);
		d.resize(n);
		t_min.resize(n);
		t_max.resize(n);

		f.read(reinterpret_cast<char*>(o.data()), sizeof(vec3) * n);
		f.read(reinterpret_cast<char*>(d.data()), sizeof(vec3) * n);
		f.read(reinterpret_cast<char*>(t_min.data()), sizeof(f32) * n);
		f.read(reinterpret_cast<char*>(t_max.data()), sizeof(f32) * n);

		if (!f.good())
		{
			LOG_ERROR("rayset: truncated file '%s'", path.c_str());
			clear();
			return false;
		}
		return true;
	}

	bool rayset::load_or_generate(rayset& out, const std::string& dir, ray_distribution dist,
		const mesh& m, const bvh2& tree, const camera& cam,
		const rayset_args& args)
	{
		const std::string name = rayset_filename(out.scene.empty() ? "scene" : out.scene, dist, args);
		const std::string path = dir.empty() ? name : dir + "/" + name;

		const std::string scene_name = out.scene;

		if (out.load(path))
		{
			LOG_INFO("rayset[%s]: loaded %u rays from %s", to_string(dist), out.size(), path.c_str());
			return true;
		}

		rayset generated = generate(dist, m, tree, cam, args);
		generated.scene = scene_name;
		out = std::move(generated);

		return out.save(path);
	}

} // namespace bvh
