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
  QuiverRenderData(Quiver const& quiver);
  ~QuiverRenderData();
};

class QuiverRenderer final : public RenderBase {
public:
  QuiverRenderer();
  virtual ~QuiverRenderer();
  virtual Status TickRender() override;
  virtual Status TickLogic() override;
  void RenderGui() final;
  virtual Status Erase(Entity entity) override;
  virtual Status Setup() override;
  virtual Status CleanUp() override;
private:
  Program prog_;
};

}