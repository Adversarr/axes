#pragma once

#include <glm/glm.hpp>

#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/program.hpp"
#include "axes/gl/render_base.hpp"
#include "axes/gl/vao.hpp"
#include "axes/core/entt.hpp"

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
  LineRenderData(Lines const& line);
  ~LineRenderData();
};

class LineRenderer final : public RenderBase {
public:
  LineRenderer();
  virtual ~LineRenderer();
  virtual Status TickRender();
  virtual Status TickLogic();
  virtual Status Erase(Entity entity);
  void RenderGui() final;
  virtual Status Setup();
  virtual Status CleanUp();

private:
  Program prog_;
};
}  // namespace ax::gl
