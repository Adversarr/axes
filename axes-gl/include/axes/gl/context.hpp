#pragma once

#include "axes/utils/common.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "render_base.hpp"
#include "window.hpp"
namespace ax::gl {

struct UiRenderEvent {};

class Context {
public:
  struct Impl;

  /****************************** Ctor Dtor ******************************/
  Context();
  Context(Context&& other) noexcept;
  AX_DECLARE_COPY_CTOR(Context, delete);
  ~Context();

  /****************************** Methods ******************************/
  Status TickLogic();
  Status TickRender();

  void AppendEntityRenderer(utils::uptr<RenderBase> renderer);

  Window& GetWindow();
  Camera& GetCamera();
  Light& GetLight();

  math::mat4r const& GetGlobalModelMatrix() const;

  void SetGlobalModelMatrix(math::mat4r const& value);

private:
  utils::uptr<Impl> impl_;
};

Context& get_context();

}  // namespace ax::gl
