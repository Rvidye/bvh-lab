#include <util/image.h>

#include <util/check.h>
#include <util/log.h>

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>

namespace bvh
{
	namespace
	{
		f32 linear_to_srgb(f32 c)
		{
			c = bvh::clamp(c, 0.0f, 1.0f);
			return c <= 0.0031308f ? 12.92f * c : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
		}

		u8 to_byte(f32 c)
		{
			return static_cast<u8>(bvh::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f);
		}

		// control points for color ramp
		constexpr int   ramp_count = 6;
		constexpr float ramp[ramp_count][3] = {
			{0.150f, 0.100f, 0.400f},
			{0.100f, 0.600f, 0.850f},
			{0.150f, 0.800f, 0.350f},
			{0.950f, 0.900f, 0.150f},
			{0.900f, 0.200f, 0.100f},
			{1.000f, 1.000f, 1.000f},
		};
	} // namespace

	vec3 image::colormap(f32 t)
	{
		t = bvh::clamp(t, 0.0f, 1.0f) * static_cast<f32>(ramp_count - 1);
		const int lo = std::min(static_cast<int>(t), ramp_count - 2);
		const f32 f = t - static_cast<f32>(lo);

		return vec3(ramp[lo][0] + (ramp[lo + 1][0] - ramp[lo][0]) * f,
					ramp[lo][1] + (ramp[lo + 1][1] - ramp[lo][1]) * f, 
					ramp[lo][2] + (ramp[lo + 1][2] - ramp[lo][2]) * f);

	}

	image image::from_counts(const std::vector<u32>& counts, u32 width, u32 height, u32 max_value)
	{
		CHECK_EQ(counts.size(), size_t(width) * height);

		if (max_value == 0)
		{
			for (u32 c : counts) max_value = bvh::max(max_value, c);
		}
		const f32 inv = max_value ? 1.0f / static_cast<f32>(max_value) : 0.0f;

		image img(width, height);
		for (u32 y = 0; y < height; ++y)
			for (u32 x = 0; x < width; ++x)
				img.set(x, y, colormap(static_cast<f32>(counts[size_t(y) * width + x]) * inv));

		return img;
	}

	bool image::write_png(const std::string& path, bool srgb) const
	{
		if (_pixels.empty())
		{
			LOG_ERROR("image: nothing to write to '%s'", path.c_str());
			return false;
		}

		std::vector<u8> bytes(_pixels.size() * 3);
		for (size_t i = 0; i < _pixels.size(); ++i)
		{
			const vec3& p = _pixels[i];
			if (srgb)
			{
				bytes[i * 3 + 0] = to_byte(linear_to_srgb(p.x));
				bytes[i * 3 + 1] = to_byte(linear_to_srgb(p.y));
				bytes[i * 3 + 2] = to_byte(linear_to_srgb(p.z));
			}
			else
			{
				bytes[i * 3 + 0] = to_byte(p.x);
				bytes[i * 3 + 1] = to_byte(p.y);
				bytes[i * 3 + 2] = to_byte(p.z);
			}
		}

		const int ok = stbi_write_png(path.c_str(), int(_width), int(_height), 3,
			bytes.data(), int(_width) * 3);
		if (!ok)
		{
			LOG_ERROR("image: stbi_write_png failed for '%s'", path.c_str());
			return false;
		}

		LOG_INFO("wrote %s (%ux%u)", path.c_str(), _width, _height);
		return true;
	}

}
