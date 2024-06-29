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
  List<LineRenderVertexData> vertices_;
  List<LineInstanceData> instance_data_;
  List<GLuint> indices_;

  Vao vao_;
  bool enable_{true};
  bool use_global_model_{true};
  LineRenderData(Lines const& line);
  ~LineRenderData();
};

class LineRenderer final : public RenderBase {
public:
  LineRenderer();
  virtual ~LineRenderer();
  virtual Status TickRender() override;
  virtual Status TickLogic() override;
  virtual Status Erase(Entity entity) override;
  void RenderGui() final;
  virtual Status Setup() override;
  virtual Status CleanUp() override;

private:
  Program prog_;
};
}  // namespace ax::gl
