#pragma once

#include <glm/glm.hpp>

#include "ax/gl/primitives/points.hpp"
#include "ax/gl/program.hpp"
#include "ax/gl/render_base.hpp"
#include "ax/gl/vao.hpp"
#include "ax/core/entt.hpp"

namespace ax::gl {

struct PointRenderVertexData {
  glm::vec3 position_;
  glm::vec4 color_;
};

struct PointRenderData {
  std::vector<PointRenderVertexData> vertices_;

  f32 point_size_{1.0};

  Vao vao_;

  bool enable_{true};

  bool use_global_model_{true};
  explicit PointRenderData(Points const& point);
  ~PointRenderData();
};

class PointRenderer final : public RenderBase {
public:
  PointRenderer();
  ~PointRenderer() override;
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
