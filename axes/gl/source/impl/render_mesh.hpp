#pragma once

#include <glm/glm.hpp>

#include "ax/core/entt.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/program.hpp"
#include "ax/gl/render_base.hpp"
#include "ax/gl/vao.hpp"

namespace ax::gl {

struct MeshRenderVertexData {
  glm::vec3 position_;
  glm::vec4 color_;
  glm::vec3 normal_;
};

struct MeshInstanceData {
  glm::vec3 position_offset_;
  glm::vec4 color_offset_;
  glm::vec3 scale_;
};

struct MeshRenderData {
  std::vector<MeshRenderVertexData> vertices_;
  std::vector<MeshInstanceData> instances_;
  std::vector<GLuint> indices_;
  Vao vao_;

  bool enable_{true};
  bool use_global_model_{false};
  bool is_flat_{false};
  bool use_lighting_{false};
  explicit MeshRenderData(Mesh const& mesh);
};

class MeshRenderer final : public RenderBase {
public:
  MeshRenderer();
  virtual ~MeshRenderer();
  virtual void TickRender() override;
  virtual void TickLogic() override;
  void RenderGui() final;
  virtual void Erase(Entity entity) override;
  virtual void Setup() override;
  virtual void CleanUp() override;

private:
  Program prog_;
};
}  // namespace ax::gl
