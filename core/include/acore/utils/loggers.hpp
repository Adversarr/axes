#pragma once
#include <fmt/format.h>
#include <spdlog/common.h>
#include <spdlog/logger.h>

namespace axes {
namespace utils {
namespace details {
std::shared_ptr<spdlog::logger> &get_default_logger();

std::shared_ptr<spdlog::logger> get_logger(const std::string &name);

void add_logger(const std::shared_ptr<spdlog::logger> &logger);

constexpr const char *default_logger_name = "AxesDefaultLogger";

void init_logger(spdlog::level::level_enum default_level,
                 bool use_spdlog_default, std::string pattern);

void cleanup_logger();
}  // namespace details

bool is_logger_inited();

inline auto get_default_logger() { return details::get_default_logger(); }

inline auto get_logger(const std::string &name) {
  return details::get_logger(name);
}

using LogLevel = spdlog::level::level_enum;

inline void set_default_log_level(LogLevel lvl) {
  get_default_logger()->set_level(lvl);
}

#ifndef AXES_LOG
// Standard Logging methods.
#  define AXES_LOG(...) \
    ((axes::utils::details::get_default_logger())->log(__VA_ARGS__))
#  define AXES_DEBUG(...) \
    AXES_LOG(spdlog::level::level_enum::debug, __VA_ARGS__)
#  define AXES_TRACE(...) \
    AXES_LOG(spdlog::level::level_enum::trace, __VA_ARGS__)
#  define AXES_INFO(...) AXES_LOG(spdlog::level::level_enum::info, __VA_ARGS__)
#  define AXES_WARN(...) AXES_LOG(spdlog::level::level_enum::warn, __VA_ARGS__)
#  define AXES_ERROR(...) AXES_LOG(spdlog::level::level_enum::err, __VA_ARGS__)
#  define AXES_CRITICAL(...) \
    AXES_LOG(spdlog::level::level_enum::critical, __VA_ARGS__)
#endif

#ifndef AXES_DEBUG_LOG
// Logging methods which only takes effect in debug mode.
#  ifndef NDEBUG
#    define AXES_DEBUG_LOG(...)                                             \
      AXES_LOG(spdlog::source_loc(__FILE__, __LINE__,                       \
                                  static_cast<const char *>(__FUNCTION__)), \
               spdlog::level::level_enum::trace, __VA_ARGS__)
#  else
#    define AXES_DEBUG_LOG(...) axes::utils::do_nothing(__VA_ARGS__);
#  endif
#endif

#ifndef AXES_DISPLAY_FUNC
#  define AXES_DISPLAY_FUNC()                                             \
    AXES_LOG(spdlog::source_loc(__FILE__, __LINE__,                       \
                                static_cast<const char *>(__FUNCTION__)), \
             spdlog::level::level_enum::info, "Called.")
#endif

}  // namespace utils
}  // namespace axes

#ifndef AXES_CHECK
#  include <iostream>
#  define AXES_CHECK(condition, ...)                                    \
    do {                                                                \
      bool retval = static_cast<bool>(condition);                       \
      if (!retval) {                                                    \
        auto msg = fmt::format(__VA_ARGS__);                            \
        if (axes::utils::is_logger_inited()) {                          \
          AXES_CRITICAL("Assertion({}) Failed: '{}', Occurs at: {}:{}", \
                        #condition, msg, __FILE__, __LINE__);           \
        } else {                                                        \
          std::cerr << "Assertion(" #condition ") Failed: " << msg      \
                    << std::endl;                                       \
          std::cerr << "Occurs at " << __FILE__ << ":" << __LINE__      \
                    << std::endl;                                       \
          std::cerr.flush();                                            \
        }                                                               \
        std::terminate();                                               \
      }                                                                 \
    } while (false)
#endif

#ifndef AXES_DEBUG_CHECK
#  if AXES_IS_DEBUG
#    define AXES_DEBUG_CHECK(condition, ...) AXES_CHECK(condition, __VA_ARGS__)
#  else
#    define AXES_DEBUG_CHECK(condition, ...) \
      axes::utils::do_nothing(__VA_ARGS__);
#  endif
#endif
#ifndef NDEBUG
#  define INSPECT(val)                                                       \
    std::cout << "[" __FILE__ << ":" << __LINE__ << "] " #val " = " << (val) \
              << std::endl;
#else
#  define INSPECT(val) axes::utils::do_nothing();
#endif
