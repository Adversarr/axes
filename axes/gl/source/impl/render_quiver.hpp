#pragma once
#include "ax/core/entt.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/program.hpp"
#include "ax/gl/render_base.hpp"
#include "ax/gl/vao.hpp"
#include <glm/glm.hpp>

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
  virtual ~QuiverRenderer();
  virtual void TickRender() override;
  virtual void TickLogic() override;
  void RenderGui() final;
  virtual void Erase(Entity entity) override;
  virtual void Setup() override;
  virtual void CleanUp() override;
private:
  Program prog_;
};

}
