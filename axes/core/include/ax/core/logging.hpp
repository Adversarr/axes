/**
 * @file echo.hpp
 * @brief This file provides a set of macros that wrap around the absl logging and checking
 *
 */

#pragma once
#include <spdlog/spdlog.h>

#include "ax/core/macros.hpp"

#define AX_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define AX_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define AX_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define AX_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define AX_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define AX_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

#define AX_CHECK(cond, ...)                                                                    \
  do {                                                                                         \
    if (!(cond)) {                                                                             \
      AX_CRITICAL("AX_CHECK ({}) failed" __VA_OPT__(": {}"), #cond, fmt::format(__VA_ARGS__)); \
      std::abort();                                                                            \
    }                                                                                          \
  } while (false)

#if (AX_IS_DEBUG)
#  define AX_DCHECK(cond, ...) AX_CHECK(cond, __VA_ARGS__)
#else
#  define AX_DCHECK(cond, ...) (void)(cond)
#endif

namespace ax {}  // namespace ax
