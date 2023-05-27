#include "axes/core/ecs/systems.hpp"

namespace axes::ecs {

void SystemBase::TickLogic() {
  // Do nothing.
}

void SystemBase::TickRender() {
  // Do nothing.
}

void SystemBase::Reset() {
  // Do nothing.
}

void SystemBase::Initialize() {
  // Do nothing.
}

std::string SystemBase::GetName() const { return "NONAME"; }

}  // namespace axes::ecs
