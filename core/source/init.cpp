#include "acore/init.hpp"
#include "acore/utils/log.hpp"
#include "acore/ecs/ecs.hpp"

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
  AXES_INFO("AXES Initialized.");
}

void shutdown() {
  axes::ecs::ResourceManager::DestroyGlobal();
  
  AXES_INFO("AXES Shutdown.");
  axes::utils::details::cleanup_logger();
}
}  // namespace axes
