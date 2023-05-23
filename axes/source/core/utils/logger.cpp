#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "axes/core/utils/common.hpp"
#include "axes/core/utils/log.hpp"

namespace axes {
namespace utils {
namespace details {

std::shared_ptr<spdlog::logger> default_logger;

static bool is_default_logger_initialized{false};

std::shared_ptr<spdlog::logger> &get_default_logger() {
  if_unlikely(default_logger == nullptr) {
    default_logger = spdlog::get(default_logger_name);
  }
  if_unlikely(default_logger.get() == nullptr) {
    throw std::logic_error(
        "You cannot use axes logger before system initialization. (remember to "
        "call `axes::init_axes()`)");
  }
  return default_logger;
}

std::shared_ptr<spdlog::logger> get_logger(const std::string &name) {
  return spdlog::get(name);
}

void add_logger(const std::shared_ptr<spdlog::logger> &logger) {
  spdlog::register_logger(logger);
}

void init_logger(spdlog::level::level_enum default_level,
                 bool use_spdlog_default, std::string pattern) {
  if (use_spdlog_default) {
    default_logger = spdlog::default_logger();
  } else {
    default_logger = spdlog::stdout_color_mt(default_logger_name);
    spdlog::set_default_logger(default_logger);
  }

  if (pattern.empty()) {
    default_logger->set_pattern("[%H:%M:%S:%e] [%^%L%$] [T:%t]: %v");
  } else {
    default_logger->set_pattern(pattern);
  }
  default_logger->set_level(default_level);
  if constexpr (axes::utils::is_debug_mode) {
    AXES_DEBUG_LOG("Default Logger Initialized.");
  } else {
    AXES_INFO("Default Logger Initialized.");
  }
  is_default_logger_initialized = true;
}

void cleanup_logger() {
  // Release the default logger.
  default_logger.reset();
  is_default_logger_initialized = false;
}

} // namespace details
bool is_logger_inited() { return details::is_default_logger_initialized; }
} // namespace utils
} // namespace axes
