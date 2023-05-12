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
  AXES_INFO("AXES Initialized.");
}

void shutdown_axes() {
  AXES_INFO("AXES Shutdown.");
  axes::utils::details::cleanup_logger();
}
}  // namespace axes
