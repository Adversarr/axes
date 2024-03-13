#pragma once

#include <entt/entity/fwd.hpp>

#include "axes/core/status.hpp"

namespace ax::gl {

class RenderBase {
public:
  virtual ~RenderBase() = default;
  virtual Status TickLogic() = 0;
  virtual Status TickRender() = 0;
  virtual Status Erase(entt::entity entity) = 0;
  virtual Status Setup() = 0;
  virtual Status CleanUp() = 0;
  virtual void RenderGui() {}
};

}  // namespace ax::gl
