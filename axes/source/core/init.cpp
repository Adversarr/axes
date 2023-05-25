#include "axes/core/init.hpp"

#include "axes/core/ecs/ecs.hpp"
#include "axes/core/utils/log.hpp"

#ifdef AXES_HAS_BACKWARD
#  include "backward.hpp"
namespace backward {

backward::SignalHandling sh;

}  // namespace backward
#endif

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
  axes::ecs::ResourceManager::WorldDestroy();
  axes::ecs::World::Destroy();
  AXES_INFO("AXES Shutdown.");
  axes::utils::details::cleanup_logger();
}
}  // namespace axes
