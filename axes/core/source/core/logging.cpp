#include "ax/core/logging.hpp"

namespace ax {

void set_log_level(loglvl lvl) {
  spdlog::set_level(lvl);
}

void set_log_pattern(const std::string& pattern) {
  spdlog::set_pattern(pattern);
}

}  // namespace ax