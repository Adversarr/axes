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
  List<QuiverRenderVertexData> vertices_;
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
  virtual Status TickRender();
  virtual Status TickLogic();
  void RenderGui() final;
  virtual Status Erase(Entity entity);
  virtual Status Setup();
  virtual Status CleanUp();
private:
  Program prog_;
};

}