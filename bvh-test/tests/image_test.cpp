#include <util/image.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <vector>

using namespace bvh;

TEST(Image, StoresAndReturnsPixels)
{
	image img(4, 3);
	EXPECT_EQ(img.width(), 4u);
	EXPECT_EQ(img.height(), 3u);

	img.set(2, 1, vec3(0.25f, 0.5f, 0.75f));
	const vec3 p = img.get(2, 1);

	EXPECT_FLOAT_EQ(p.x, 0.25f);
	EXPECT_FLOAT_EQ(p.y, 0.5f);
	EXPECT_FLOAT_EQ(p.z, 0.75f);
}

TEST(Image, PixelsAreIndependent)
{
	image img(5, 4);
	for (u32 y = 0; y < 4; ++y)
		for (u32 x = 0; x < 5; ++x)
			img.set(x, y, vec3(f32(x), f32(y), 0.0f));

	for (u32 y = 0; y < 4; ++y)
		for (u32 x = 0; x < 5; ++x)
		{
			const vec3 p = img.get(x, y);
			ASSERT_FLOAT_EQ(p.x, f32(x));
			ASSERT_FLOAT_EQ(p.y, f32(y));
		}
}

TEST(Image, ColormapIsClampedAndContinuous)
{
	for (f32 t = -0.5f; t <= 1.5f; t += 0.05f)
	{
		const vec3 c = image::colormap(t);
		ASSERT_GE(c.x, 0.0f); ASSERT_LE(c.x, 1.0f);
		ASSERT_GE(c.y, 0.0f); ASSERT_LE(c.y, 1.0f);
		ASSERT_GE(c.z, 0.0f); ASSERT_LE(c.z, 1.0f);
	}
}

TEST(Image, ColormapEndpointsDiffer)
{
	const vec3 lo = image::colormap(0.0f);
	const vec3 hi = image::colormap(1.0f);
	EXPECT_GT(length(hi - lo), 0.5f) << "colormap must actually span a range";
}

TEST(Image, HeatmapScalesToMaximum)
{
	const std::vector<u32> counts(16, 7u);
	const image img = image::from_counts(counts, 4, 4);

	const vec3 expected = image::colormap(1.0f);
	const vec3 actual   = img.get(0, 0);

	EXPECT_NEAR(actual.x, expected.x, 1e-5f);
	EXPECT_NEAR(actual.y, expected.y, 1e-5f);
	EXPECT_NEAR(actual.z, expected.z, 1e-5f);
}

TEST(Image, HeatmapExplicitMaximumKeepsScaleStable)
{
	std::vector<u32> counts(4, 0u);
	counts[0] = 5u;

	const image img = image::from_counts(counts, 2, 2, 10u);
	const vec3 expected = image::colormap(0.5f);
	const vec3 actual   = img.get(0, 0);

	EXPECT_NEAR(actual.x, expected.x, 1e-5f);
	EXPECT_NEAR(actual.y, expected.y, 1e-5f);
	EXPECT_NEAR(actual.z, expected.z, 1e-5f);
}

TEST(Image, HeatmapOfAllZeroesDoesNotDivideByZero)
{
	const std::vector<u32> counts(9, 0u);
	const image img = image::from_counts(counts, 3, 3);

	const vec3 c = img.get(1, 1);
	EXPECT_FALSE(std::isnan(c.x));
	EXPECT_FALSE(std::isnan(c.y));
	EXPECT_FALSE(std::isnan(c.z));
}

TEST(Image, WritesPngFile)
{
	const std::filesystem::path path =
	    std::filesystem::temp_directory_path() / "bvhlab_image_test.png";
	std::filesystem::remove(path);

	image img(8, 8);
	for (u32 y = 0; y < 8; ++y)
		for (u32 x = 0; x < 8; ++x)
			img.set(x, y, vec3(f32(x) / 7.0f, f32(y) / 7.0f, 0.5f));

	ASSERT_TRUE(img.write_png(path.string()));
	ASSERT_TRUE(std::filesystem::exists(path));
	EXPECT_GT(std::filesystem::file_size(path), 0u);

	std::filesystem::remove(path);
}

TEST(Image, WritingAnEmptyImageFails)
{
	const image img;
	EXPECT_FALSE(img.write_png("should-not-be-created.png"));
	EXPECT_FALSE(std::filesystem::exists("should-not-be-created.png"));
}
