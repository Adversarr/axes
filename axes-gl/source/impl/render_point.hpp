#pragma once

#include <glm/glm.hpp>

#include "axes/gl/primitives/points.hpp"
#include "axes/gl/program.hpp"
#include "axes/gl/render_base.hpp"
#include "axes/gl/vao.hpp"
#include "axes/core/entt.hpp"

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
  PointRenderData(Points const& point);
  ~PointRenderData();
};

class PointRenderer final : public RenderBase {
public:
  PointRenderer();
  virtual ~PointRenderer();
  virtual Status TickRender();
  virtual Status TickLogic();
  void RenderGui() final;
  virtual Status Erase(Entity entity);
  virtual Status Setup();
  virtual Status CleanUp();

private:
  Program prog_;
};
}  // namespace ax::gl
