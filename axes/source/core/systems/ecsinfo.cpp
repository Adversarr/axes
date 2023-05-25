#include "axes/core/systems/ecsinfo.hpp"

#include "axes/core/utils/log.hpp"

namespace axes {

void EcsInfoSystem::TickLogic() {
  ecs::World world;
  AXES_INFO("In Ecs System: #ent={}", world.GetEntities().size());
  AXES_INFO("Registered Components count={}",
            world.GetRegisteredComponents().size());
  for (const auto &cmpt : world.GetRegisteredComponents()) {
    AXES_INFO("For Component {}, #ent={}", cmpt.ti_.name(),
              cmpt.query_all_().size());
  }
}

}  // namespace axes
