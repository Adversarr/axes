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

#define AX_CHECK(cond, msg, ...)                                                   \
  do {                                                                             \
    if (!(cond)) {                                                                 \
      AX_CRITICAL("AX_CHECK ({}) failed: " msg, #cond __VA_OPT__(, ) __VA_ARGS__); \
      std::abort();                                                                \
    }                                                                              \
  } while (false)

#if (AX_IS_DEBUG)
#  define AX_DCHECK(cond, ...) AX_CHECK(cond, __VA_ARGS__)
#else
#  define AX_DCHECK(cond, ...) (void)(cond)
#endif

namespace ax {

// Possible values are: trace, debug, info, warn, err, critical, off
using loglvl = spdlog::level::level_enum;

void set_log_level(loglvl lvl);

void set_log_pattern(const std::string& pattern);

}  // namespace ax
