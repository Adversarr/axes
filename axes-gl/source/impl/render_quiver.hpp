#include "axes/core/entt.hpp"
#include "axes/gl/primitives/quiver.hpp"
#include "axes/gl/program.hpp"
#include "axes/gl/render_base.hpp"
#include "axes/gl/vao.hpp"
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