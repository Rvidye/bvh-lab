#pragma once

#include <bvh.h>
#include <build/bvh2_builder.h>
#include <core/ray.h>
#include <core/vec.h>
#include <util/camera.h>
#include <util/mesh.h>

#include <string>
#include <vector>

namespace bvh {
	enum class ray_distribution
	{
		primary,     // camera rays, one per pixel
		shadow_ao,   // short hemisphere rays from primary hits, bounded t_max
		reflection,  // mirror direction from primary hits -- coherent but not camera-aligned
		diffuse_1,   // one cosine-weighted bounce from primary hits
		diffuse_n,   // several bounces deep; the incoherent case that actually occurs in a path tracer
		incoherent,  // random origins in the scene bounds, uniform directions -- the worst case
	};

	const char* to_string(ray_distribution d);
	bool        ray_distribution_from_string(const std::string& s, ray_distribution& out);

	// All six, in a fixed order, for iterating the workload axis.
	constexpr ray_distribution all_ray_distributions[] = {
		ray_distribution::primary,    ray_distribution::shadow_ao,
		ray_distribution::reflection, ray_distribution::diffuse_1,
		ray_distribution::diffuse_n,  ray_distribution::incoherent,
	};

	struct rayset_args
	{
		u32 width{ 512 };
		u32 height{ 512 };
		u32 bounces{ 3 };
		f32 ao_radius_fraction{ 0.1f };
		u32 incoherent_count{ 262144 };
		u64 seed{ 0x9e3779b97f4a7c15ull };
	};

	// ray batch.
	class rayset
	{
	public:
		std::vector<vec3> o;
		std::vector<vec3> d;
		std::vector<f32>  t_min;
		std::vector<f32>  t_max;

		ray_distribution distribution{ ray_distribution::primary };
		std::string      scene;
		u64              seed{ 0 };

		u32  size() const { return static_cast<u32>(o.size()); }
		bool empty() const { return o.empty(); }
		void clear();
		void reserve(u32 n);

		ray  get(u32 i) const { return ray(o[i], d[i], t_min[i], t_max[i]); }
		void push(const ray& r);

		static rayset generate(ray_distribution dist, const mesh& m, const bvh2& tree,
			const camera& cam, const rayset_args& args = {});

		bool save(const std::string& path) const;
		bool load(const std::string& path);

		u64 hash() const;

		static bool load_or_generate(rayset& out, const std::string& dir,
			ray_distribution dist, const mesh& m, const bvh2& tree,
			const camera& cam, const rayset_args& args = {});
	};

	// on-disk name, so load_or_generate and the harness agree.
	std::string rayset_filename(const std::string& scene, ray_distribution d, const rayset_args& a);

} // namespace bvh
