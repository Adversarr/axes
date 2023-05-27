#include "axes/core/systems/ecsinfo.hpp"

#include "axes/core/utils/log.hpp"

#include "absl/cleanup/cleanup.h"

namespace axes {

void EcsInfoSystemRunningStat::InitResource() {
  auto stat = ecs::RMan::Construct<EcsInfoSystemRunningStat>();
  stat->enable_ = false;
  stat.Publish();
}

void EcsInfoSystem::TickLogic() {
  auto stat = ecs::Rc<EcsInfoSystemRunningStat>{}.MakeValid();
  if (!stat->enable_) {
    return;
  }
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
