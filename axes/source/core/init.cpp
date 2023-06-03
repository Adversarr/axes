#include "axes/core/init.hpp"

#include "axes/core/ecs/ecs.hpp"
#include <absl/debugging/failure_signal_handler.h>
#include "axes/core/utils/log.hpp"

// #ifdef AXES_HAS_BACKWARD
// #  include "backward.hpp"
// namespace backward {
//
// }  // namespace backward
// #endif

namespace axes {

void init() {
  axes::utils::details::init_logger(
#ifdef NDEBUG
      spdlog::level::info
#else
      spdlog::level::debug
#endif
      ,
      true, "");
  absl::InstallFailureSignalHandler({});
  AXES_INFO("AXES Initialized.");
}

void shutdown() {
  axes::ecs::RMan::DestroyAll();
  axes::ecs::World::DestroyAll();
  AXES_INFO("AXES Shutdown.");
  axes::utils::details::cleanup_logger();
}
}  // namespace axes
