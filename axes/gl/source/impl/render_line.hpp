#pragma once

#include <glm/glm.hpp>

#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/program.hpp"
#include "ax/gl/render_base.hpp"
#include "ax/gl/vao.hpp"
#include "ax/core/entt.hpp"

namespace ax::gl {

struct LineRenderVertexData {
  glm::vec3 position_;
  glm::vec4 color_;
};

struct LineInstanceData {
  glm::vec3 offset_;
  glm::vec4 color_;
};

struct LineRenderData {
  std::vector<LineRenderVertexData> vertices_;
  std::vector<LineInstanceData> instance_data_;
  std::vector<GLuint> indices_;

  Vao vao_;
  bool enable_{true};
  bool use_global_model_{true};
  bool dim_far_away_from_center_{true};
  explicit LineRenderData(Lines const& line);
  ~LineRenderData();
};

class LineRenderer final : public RenderBase {
public:
  LineRenderer();
  ~LineRenderer() override;
  virtual void TickRender() override;
  virtual void TickLogic() override;
  virtual void Erase(Entity entity) override;
  void RenderGui() override;
  virtual void Setup() override;
  virtual void CleanUp() override;

private:
  Program prog_;
};
}  // namespace ax::gl
