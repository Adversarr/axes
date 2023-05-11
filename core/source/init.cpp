#include "acore/init.hpp"

#include "acore/utils/log.hpp"
namespace axes {

void init_axes() {
  axes::utils::details::init_logger(
#ifdef NDEBUG
      spdlog::level::info
#else
      spdlog::level::debug
#endif
      ,
      true, "");
}

void shutdown_axes() {
  axes::utils::details::cleanup_logger();
}
}  // namespace axes
