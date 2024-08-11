#pragma once

#include "ax/utils/common.hpp"
#include "camera.hpp"
#include "events.hpp"  // IWYU pragma: export
#include "light.hpp"
#include "render_base.hpp"
#include "window.hpp"

namespace ax::gl {

struct ContextShouldShutdownEvent {};

class Context {
public:
  struct Impl;

  /****************************** Ctor Dtor ******************************/
  Context();
  Context(Context&& other) noexcept;
  AX_DECLARE_COPY_CTOR(Context, delete);
  ~Context();

  /****************************** Methods ******************************/
  void TickLogic();
  void TickRender();
  void Initialize();
  bool ShouldClose() const;

  /**
   * Add a custom renderer to the render pipeline.
   * @param renderer RenderBase
   */
  void AppendEntityRenderer(std::unique_ptr<RenderBase> renderer);

  Window& GetWindow();
  Camera& GetCamera();
  Light& GetLight();

  math::mat4f const& GetGlobalModelMatrix() const;

  void SetGlobalModelMatrix(math::mat4f const& value);

private:
  std::unique_ptr<Impl> impl_;
};

Context& get_context();

}  // namespace ax::gl
