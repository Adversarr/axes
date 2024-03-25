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
  glm::vec4 color_;
  glm::vec3 normal_;
};

struct MeshInstanceData {
  glm::vec3 position_offset_;
  glm::vec4 color_offset_;
};

struct MeshRenderData {
  List<MeshRenderVertexData> vertices_;
  List<MeshInstanceData> instances_;
  List<GLuint> indices_;
  Vao vao_;

  bool enable_{true};
  bool use_global_model_{false};
  bool is_flat_{false};
  bool use_lighting_{false};
  MeshRenderData(Mesh const& line);
};

class MeshRenderer final : public RenderBase {
public:
  MeshRenderer();
  virtual ~MeshRenderer();
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
