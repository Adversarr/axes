#pragma once

#include <glm/glm.hpp>

#include "axes/core/entt.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/program.hpp"
#include "axes/gl/render_base.hpp"
#include "axes/gl/vao.hpp"

namespace ax::gl {

struct MeshRenderVertexData {
  glm::vec3 position_;
  glm::vec3 normal_;
  glm::vec4 color_;
};

struct MeshInstanceData {
  glm::vec3 position_offset_;
  glm::vec4 color_offset_;
};

struct MeshRenderData {
  std::vector<MeshRenderVertexData> vertices_;
  std::vector<MeshInstanceData> instances_;
  std::vector<GLuint> indices_;
  math::vec4f wireframe_color_{0.0f, 0.0f, 0.0f, 1.0f};
  Vao vao_;

  bool is_flat_{false};
  bool use_lighting_{false};
  bool render_wireframe_{false};
  MeshRenderData(Mesh const& line);
};

class MeshRenderer final : public RenderBase {
public:
  MeshRenderer();
  virtual ~MeshRenderer();
  virtual Status TickRender();
  virtual Status TickLogic();
  virtual Status Erase(Entity entity);
  virtual Status Setup();
  virtual Status CleanUp();

private:
  Program prog_;
};
}  // namespace ax::gl
