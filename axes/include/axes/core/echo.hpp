/**
 * @file echo.hpp
 * @brief This file provides a set of macros that wrap around the absl logging and checking
 *
 */

#pragma once
#include <absl/log/absl_check.h>   // IWYU pragma: export
#include <absl/log/absl_log.h>     // IWYU pragma: export
#include <absl/log/die_if_null.h>  // IWYU pragma: export

#define AX_DIE_IF_NULL(...) ABSL_DIE_IF_NULL(__VA_ARGS__)

#define AX_CHECK(COND) ABSL_CHECK(COND)
#define AX_QCHECK(COND) ABSL_QCHECK(COND)
#define AX_PCHECK(COND) ABSL_PCHECK(COND)
#define AX_DCHECK(COND) ABSL_DCHECK(COND)

#define AX_CHECK_EQ(val1, val2) ABSL_CHECK_EQ(val1, val2)
#define AX_CHECK_NE(val1, val2) ABSL_CHECK_NE(val1, val2)
#define AX_CHECK_LT(val1, val2) ABSL_CHECK_LT(val1, val2)
#define AX_CHECK_LE(val1, val2) ABSL_CHECK_LE(val1, val2)
#define AX_CHECK_GT(val1, val2) ABSL_CHECK_GT(val1, val2)
#define AX_CHECK_GE(val1, val2) ABSL_CHECK_GE(val1, val2)

#define AX_QCHECK_EQ(val1, val2) ABSL_QCHECK_EQ(val1, val2)
#define AX_QCHECK_NE(val1, val2) ABSL_QCHECK_NE(val1, val2)
#define AX_QCHECK_LT(val1, val2) ABSL_QCHECK_LT(val1, val2)
#define AX_QCHECK_LE(val1, val2) ABSL_QCHECK_LE(val1, val2)
#define AX_QCHECK_GT(val1, val2) ABSL_QCHECK_GT(val1, val2)
#define AX_QCHECK_GE(val1, val2) ABSL_QCHECK_GE(val1, val2)

#define AX_DCHECK_EQ(val1, val2) ABSL_DCHECK_EQ(val1, val2)
#define AX_DCHECK_NE(val1, val2) ABSL_DCHECK_NE(val1, val2)
#define AX_DCHECK_LT(val1, val2) ABSL_DCHECK_LT(val1, val2)
#define AX_DCHECK_LE(val1, val2) ABSL_DCHECK_LE(val1, val2)
#define AX_DCHECK_GT(val1, val2) ABSL_DCHECK_GT(val1, val2)
#define AX_DCHECK_GE(val1, val2) ABSL_DCHECK_GE(val1, val2)

#define AX_CHECK_OK(val) ABSL_CHECK_OK(val)
#define AX_QCHECK_OK(val) ABSL_QCHECK_OK(val)
#define AX_DCHECK_OK(val) ABSL_DCHECK_OK(val)

#define AX_CHECK_STREQ(val1, val2) ABSL_CHECK_STREQ(val1, val2)
#define AX_CHECK_STRNE(val1, val2) ABSL_CHECK_STRNE(val1, val2)
#define AX_CHECK_STRCASEEQ(val1, val2) ABSL_CHECK_STRCASEEQ(val1, val2)
#define AX_CHECK_STRCASENE(val1, val2) ABSL_CHECK_STRCASENE(val1, val2)

#define AX_QCHECK_STREQ(val1, val2) ABSL_QCHECK_STREQ(val1, val2)
#define AX_QCHECK_STRNE(val1, val2) ABSL_QCHECK_STRNE(val1, val2)
#define AX_QCHECK_STRCASEEQ(val1, val2) ABSL_QCHECK_STRCASEEQ(val1, val2)
#define AX_QCHECK_STRCASENE(val1, val2) ABSL_QCHECK_STRCASENE(val1, val2)

#define AX_DCHECK_STREQ(val1, val2) ABSL_DCHECK_STREQ(val1, val2)
#define AX_DCHECK_STRNE(val1, val2) ABSL_DCHECK_STRNE(val1, val2)
#define AX_DCHECK_STRCASEEQ(val1, val2) ABSL_DCHECK_STRCASEEQ(val1, val2)
#define AX_DCHECK_STRCASENE(val1, val2) ABSL_DCHECK_STRCASENE(val1, val2)

#define AX_LOG(LVL) ABSL_LOG(LVL)
#define AX_PLOG(LVL) ABSL_PLOG(LVL)
#define AX_DLOG(LVL) ABSL_DLOG(LVL)

#define AX_LOG_IF(LVL, COND) ABSL_LOG_IF(LVL, COND)
#define AX_PLOG_IF(LVL, COND) ABSL_PLOG_IF(LVL, COND)
#define AX_DLOG_IF(LVL, COND) ABSL_DLOG_IF(LVL, COND)

#define AX_LOG_EVERY_N(LVL, N) ABSL_LOG_EVERY_N(LVL, N)
#define AX_PLOG_EVERY_N(LVL, N) ABSL_PLOG_EVERY_N(LVL, N)
#define AX_DLOG_EVERY_N(LVL, N) ABSL_DLOG_EVERY_N(LVL, N)

#define AX_LOG_FIRST_N(LVL, N) ABSL_LOG_FIRST_N(LVL, N)
#define AX_PLOG_FIRST_N(LVL, N) ABSL_PLOG_FIRST_N(LVL, N)
#define AX_DLOG_FIRST_N(LVL, N) ABSL_DLOG_FIRST_N(LVL, N)

#define AX_LOG_EVERY_POW_2(LVL) ABSL_LOG_EVERY_POW_2(LVL)
#define AX_PLOG_EVERY_POW_2(LVL) ABSL_PLOG_EVERY_POW_2(LVL)
#define AX_DLOG_EVERY_POW_2(LVL) ABSL_DLOG_EVERY_POW_2(LVL)

#define AX_LOG_EVERY_N_SEC(LVL, N) ABSL_LOG_EVERY_N_SEC(LVL, N)
#define AX_PLOG_EVERY_N_SEC(LVL, N) ABSL_PLOG_EVERY_N_SEC(LVL, N)
#define AX_DLOG_EVERY_N_SEC(LVL, N) ABSL_DLOG_EVERY_N_SEC(LVL, N)

#define AX_LOG_IF_EVERY_N(LVL, COND, N) ABSL_LOG_IF_EVERY_N(LVL, COND, N)
#define AX_PLOG_IF_EVERY_N(LVL, COND, N) ABSL_PLOG_IF_EVERY_N(LVL, COND, N)
#define AX_DLOG_IF_EVERY_N(LVL, COND, N) ABSL_DLOG_IF_EVERY_N(LVL, COND, N)

#define AX_LOG_IF_FIRST_N(LVL, COND, N) ABSL_LOG_IF_FIRST_N(LVL, COND, N)
#define AX_PLOG_IF_FIRST_N(LVL, COND, N) ABSL_PLOG_IF_FIRST_N(LVL, COND, N)
#define AX_DLOG_IF_FIRST_N(LVL, COND, N) ABSL_DLOG_IF_FIRST_N(LVL, COND, N)

#define AX_LOG_IF_EVERY_POW_2(LVL, COND) ABSL_LOG_IF_EVERY_POW_2(LVL, COND)
#define AX_PLOG_IF_EVERY_POW_2(LVL, COND) ABSL_PLOG_IF_EVERY_POW_2(LVL, COND)
#define AX_DLOG_IF_EVERY_POW_2(LVL, COND) ABSL_DLOG_IF_EVERY_POW_2(LVL, COND)

#define AX_LOG_IF_EVERY_N_SEC(LVL, COND, N) ABSL_LOG_IF_EVERY_N_SEC(LVL, COND, N)
#define AX_PLOG_IF_EVERY_N_SEC(LVL, COND, N) ABSL_PLOG_IF_EVERY_N_SEC(LVL, COND, N)
#define AX_DLOG_IF_EVERY_N_SEC(LVL, COND, N) ABSL_DLOG_IF_EVERY_N_SEC(LVL, COND, N)

#define AX_UNREACHABLE() ABSL_UNREACHABLE()

namespace ax {}  // namespace ax
