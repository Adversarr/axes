#pragma once
#include <glm/glm.hpp>

#include "ax/core/entt.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/program.hpp"
#include "ax/gl/render_base.hpp"
#include "ax/gl/vao.hpp"

namespace ax::gl {

struct QuiverRenderVertexData {
  glm::vec3 position_;
  glm::vec4 color_;
};

struct QuiverRenderData {
  std::vector<QuiverRenderVertexData> vertices_;
  Vao vao_;
  bool enable_{true};
  bool use_global_model_{true};
  explicit QuiverRenderData(Quiver const& quiver);
  ~QuiverRenderData();
};

class QuiverRenderer final : public RenderBase {
public:
  QuiverRenderer();
  ~QuiverRenderer() override;
  void TickRender() override;
  void TickLogic() override;
  void RenderGui() override;
  void Erase(Entity entity) override;
  void Setup() override;
  void CleanUp() override;

private:
  Program prog_;
};

}  // namespace ax::gl
