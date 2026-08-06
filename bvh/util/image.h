#pragma once

#include <bvh.h>
#include <core/vec.h>

#include <string>
#include <vector>

namespace bvh 
{
	// linear space rgb framebuffer

	class image
	{
	public:
		image() = default;
		image(u32 width, u32 height) : _width(width), _height(height), _pixels(size_t(width)* height) {}

		void set(u32 x, u32 y, const vec3& rgb) { _pixels[size_t(y) * _width + x] = rgb; }
		vec3 get(u32 x, u32 y) const { return _pixels[size_t(y) * _width + x]; }

		u32 width() const { return _width; }
		u32 height() const { return _height; }

		bool write_png(const std::string& path, bool srgb = true) const;

		// per-pixel counter visualization
		static image from_counts(const std::vector<u32>& counts, u32 width, u32 height, u32 max_value = 0);

		// perceptually monotonic colormap
		static vec3 colormap(f32 t);

	private:
		u32 _width{ 0 };
		u32 _height{ 0 };
		std::vector<vec3> _pixels;
	};

} // namespace bvh

