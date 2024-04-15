#pragma once

#include "ax/utils/common.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "render_base.hpp"
#include "window.hpp"
namespace ax::gl {

struct UiRenderEvent {};
struct ContextDestroyEvent {};
struct ContextInitEvent {};
struct MainMenuBarRenderEvent {};


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

  /**
   * Add a custom renderer to the render pipeline.
   * @param renderer RenderBase
   */
  void AppendEntityRenderer(UPtr<RenderBase> renderer);

  Window& GetWindow();
  Camera& GetCamera();
  Light& GetLight();

  math::mat4f const& GetGlobalModelMatrix() const;

  void SetGlobalModelMatrix(math::mat4f const& value);

private:
  UPtr<Impl> impl_;
};

Context& get_context();

}  // namespace ax::gl
