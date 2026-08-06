#include <util/metrics.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace bvh;

namespace {

std::string temp_csv(const char* stem)
{
	const std::filesystem::path p =
	    std::filesystem::temp_directory_path() / (std::string("bvhlab_") + stem + ".csv");
	std::filesystem::remove(p);
	return p.string();
}

std::vector<std::string> read_lines(const std::string& path)
{
	std::vector<std::string> lines;
	std::ifstream in(path);
	std::string line;
	while (std::getline(in, line))
	{
		if (!line.empty() && line.back() == '\r') line.pop_back();
		if (!line.empty()) lines.push_back(line);
	}
	return lines;
}

} // namespace

TEST(Metrics, WritesHeaderThenRow)
{
	const std::string path = temp_csv("header");

	metrics m;
	m.set("scene", "teapot.obj");
	m.set("triangles", 1024u);
	m.set("trace_s", 0.5, 4);

	ASSERT_TRUE(m.flush(path));

	const auto lines = read_lines(path);
	ASSERT_EQ(lines.size(), 2u);
	EXPECT_EQ(lines[0], "scene,triangles,trace_s");
	EXPECT_EQ(lines[1], "teapot.obj,1024,0.5000");

	std::filesystem::remove(path);
}

TEST(Metrics, PreservesInsertionOrderAsColumnOrder)
{
	const std::string path = temp_csv("order");

	metrics m;
	m.set("zebra", 1);
	m.set("apple", 2);
	m.set("mango", 3);
	ASSERT_TRUE(m.flush(path));

	const auto lines = read_lines(path);
	ASSERT_EQ(lines.size(), 2u);
	EXPECT_EQ(lines[0], "zebra,apple,mango");
	EXPECT_EQ(lines[1], "1,2,3");

	std::filesystem::remove(path);
}

TEST(Metrics, AppendsWithoutRepeatingHeader)
{
	const std::string path = temp_csv("append");

	for (int i = 0; i < 3; ++i)
	{
		metrics m;
		m.set("run", i);
		m.set("value", i * 10);
		ASSERT_TRUE(m.flush(path));
	}

	const auto lines = read_lines(path);
	ASSERT_EQ(lines.size(), 4u); // one header + three rows
	EXPECT_EQ(lines[0], "run,value");
	EXPECT_EQ(lines[1], "0,0");
	EXPECT_EQ(lines[3], "2,20");

	std::filesystem::remove(path);
}

// A shifted CSV is worse than no CSV, because it still plots. flush() must
// refuse rather than silently misalign columns.
TEST(Metrics, RefusesToAppendOnHeaderMismatch)
{
	const std::string path = temp_csv("mismatch");

	metrics a;
	a.set("alpha", 1);
	a.set("beta", 2);
	ASSERT_TRUE(a.flush(path));

	metrics b;
	b.set("alpha", 3);
	b.set("gamma", 4); // different column set
	EXPECT_FALSE(b.flush(path));

	// The bad row must not have been written.
	const auto lines = read_lines(path);
	EXPECT_EQ(lines.size(), 2u);

	std::filesystem::remove(path);
}

TEST(Metrics, SetOverwritesInPlaceKeepingColumnPosition)
{
	const std::string path = temp_csv("overwrite");

	metrics m;
	m.set("a", 1);
	m.set("b", 2);
	m.set("a", 99); // overwrite, must not append a fourth column
	ASSERT_TRUE(m.flush(path));

	const auto lines = read_lines(path);
	ASSERT_EQ(lines.size(), 2u);
	EXPECT_EQ(lines[0], "a,b");
	EXPECT_EQ(lines[1], "99,2");

	std::filesystem::remove(path);
}

TEST(Metrics, EscapesValuesContainingCommas)
{
	const std::string path = temp_csv("escape");

	metrics m;
	m.set("note", "a,b");
	m.set("plain", "xyz");
	ASSERT_TRUE(m.flush(path));

	const auto lines = read_lines(path);
	ASSERT_EQ(lines.size(), 2u);
	EXPECT_EQ(lines[1], "\"a,b\",xyz");

	std::filesystem::remove(path);
}

TEST(Metrics, EmptyFlushIsANoOp)
{
	const std::string path = temp_csv("empty");

	metrics m;
	EXPECT_TRUE(m.flush(path));
	EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(Metrics, DecimalPlacesAreRespected)
{
	const std::string path = temp_csv("decimals");

	metrics m;
	m.set("two", 1.23456, 2);
	// 1234.5 rounds to 1234, not 1235: printf uses round-half-to-EVEN, not
	// round-half-away-from-zero. Worth pinning, because a metric that lands on
	// a .5 boundary will otherwise look off by one in the CSV.
	m.set("zero", 1234.5, 0);
	m.set("odd", 1235.5, 0);
	ASSERT_TRUE(m.flush(path));

	const auto lines = read_lines(path);
	ASSERT_EQ(lines.size(), 2u);
	EXPECT_EQ(lines[1], "1.23,1234,1236");

	std::filesystem::remove(path);
}
