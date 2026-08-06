#pragma once

#include <util/log.h>

#define CHECK(x)                                                               \
	do {                                                                       \
		if (!(x)) LOG_FATAL("check failed: %s", #x);                           \
	} while (false)

#define CHECK_MSG(x, ...)                                                      \
	do {                                                                       \
		if (!(x)) LOG_FATAL(__VA_ARGS__);                                      \
	} while (false)

#define BVH_CHECK_OP(a, b, op)                                                 \
	do {                                                                       \
		const auto _va = (a);                                                  \
		const auto _vb = (b);                                                  \
		if (!(_va op _vb))                                                     \
			LOG_FATAL("check failed: %s %s %s", #a, #op, #b);                  \
	} while (false)

#define CHECK_EQ(a, b) BVH_CHECK_OP(a, b, ==)
#define CHECK_NE(a, b) BVH_CHECK_OP(a, b, !=)
#define CHECK_LT(a, b) BVH_CHECK_OP(a, b, <)
#define CHECK_LE(a, b) BVH_CHECK_OP(a, b, <=)
#define CHECK_GT(a, b) BVH_CHECK_OP(a, b, >)
#define CHECK_GE(a, b) BVH_CHECK_OP(a, b, >=)

#ifdef BVH_DEBUG_BUILD

#define DCHECK(x)      CHECK(x)
#define DCHECK_EQ(a, b) CHECK_EQ(a, b)
#define DCHECK_NE(a, b) CHECK_NE(a, b)
#define DCHECK_LT(a, b) CHECK_LT(a, b)
#define DCHECK_LE(a, b) CHECK_LE(a, b)
#define DCHECK_GT(a, b) CHECK_GT(a, b)
#define DCHECK_GE(a, b) CHECK_GE(a, b)

#else

// Expand to a statement that swallows the trailing semicolon
#define BVH_EMPTY_CHECK do {} while (false)

#define DCHECK(x)       BVH_EMPTY_CHECK
#define DCHECK_EQ(a, b) BVH_EMPTY_CHECK
#define DCHECK_NE(a, b) BVH_EMPTY_CHECK
#define DCHECK_LT(a, b) BVH_EMPTY_CHECK
#define DCHECK_LE(a, b) BVH_EMPTY_CHECK
#define DCHECK_GT(a, b) BVH_EMPTY_CHECK
#define DCHECK_GE(a, b) BVH_EMPTY_CHECK

#endif

