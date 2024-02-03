#pragma once

#include "axes/gl/window.hpp"
#include "axes/utils/common.hpp"
#include "render_base.hpp"
namespace ax::gl {

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

private:
  utils::uptr<Impl> impl_;
};

Context& get_context();

}  // namespace ax::gl
