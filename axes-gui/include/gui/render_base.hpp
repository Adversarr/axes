#pragma once

#include <entt/entity/fwd.hpp>

#include "axes/core/status.hpp"

namespace ax::gui {

class RenderBase {
public:
  virtual ~RenderBase() = default;
  virtual Status TickRender() = 0;
  virtual Status ClearAll() = 0;
  virtual Status Erase(entt::entity entity) = 0;
  virtual Status Setup() = 0;
  virtual Status CleanUp() = 0;
};

}  // namespace ax::gui
