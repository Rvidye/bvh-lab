#include <core/rng.h>

#include <gtest/gtest.h>

#include <set>
#include <vector>

using namespace bvh;

TEST(RNG, SameSeedGivesSameSequence)
{
	rng a(1234u, 1u);
	rng b(1234u, 1u);

	for (int i = 0; i < 64; ++i) EXPECT_EQ(a.next_u32(), b.next_u32());
}

TEST(RNG, DifferentSeedsDiverge)
{
	rng a(1234u, 1u);
	rng b(1235u, 1u);

	int same = 0;
	for (int i = 0; i < 64; ++i) same += (a.next_u32() == b.next_u32()) ? 1 : 0;
	EXPECT_LT(same, 4);
}

TEST(RNG, DifferentStreamsDiverge)
{
	rng a(1234u, 1u);
	rng b(1234u, 2u);

	int same = 0;
	for (int i = 0; i < 64; ++i) same += (a.next_u32() == b.next_u32()) ? 1 : 0;
	EXPECT_LT(same, 4);
}

TEST(RNG, FloatsAreInUnitInterval)
{
	rng r(99u, 1u);
	for (int i = 0; i < 100000; ++i)
	{
		const f32 v = r.next_f32();
		ASSERT_GE(v, 0.0f);
		ASSERT_LT(v, 1.0f); // half-open: 1.0 must never appear
	}
}

TEST(RNG, FloatsAreRoughlyUniform)
{
	rng r(7u, 1u);
	int buckets[10] = {0};
	const int n = 100000;

	for (int i = 0; i < n; ++i)
	{
		const int b = static_cast<int>(r.next_f32() * 10.0f);
		ASSERT_GE(b, 0);
		ASSERT_LT(b, 10);
		++buckets[b];
	}

	for (int b = 0; b < 10; ++b)
	{
		EXPECT_GT(buckets[b], n / 20) << "bucket " << b << " underfilled";
		EXPECT_LT(buckets[b], n / 5) << "bucket " << b << " overfilled";
	}
}

TEST(RNG, DoesNotRepeatQuickly)
{
	rng r(42u, 1u);
	std::set<u32> seen;
	for (int i = 0; i < 10000; ++i) seen.insert(r.next_u32());
	EXPECT_GT(seen.size(), 9990u);
}
