// bvh-test: unit tests for bvh-lab.

#include <bvh.h>
#include <util/log.h>

#include <gtest/gtest.h>

#include <cstdio>

int main(int argc, char** argv)
{
	printf("=== bvh-lab unit tests ===\n\n");
	bvh::log_init(bvh::log_level::error);
	testing::InitGoogleTest(&argc, argv);
	const int result = RUN_ALL_TESTS();
	bvh::log_shutdown();
	return result;
}
