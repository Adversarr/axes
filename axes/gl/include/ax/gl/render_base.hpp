#pragma once

#include <entt/entity/fwd.hpp>

namespace ax::gl {

class RenderBase {
public:
  virtual ~RenderBase() = default;
  virtual void TickLogic() = 0;
  virtual void TickRender() = 0;
  virtual void Erase(entt::entity entity) = 0;
  virtual void Setup() = 0;
  virtual void CleanUp() = 0;
  virtual void RenderGui() {}
};

}  // namespace ax::gl
