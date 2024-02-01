#pragma once

#include <vector>

#include "axes/utils/common.hpp"
#include "render_base.hpp"
#include "window.hpp"
namespace ax::gui {

class Context {
public:
  struct Impl;
  Context();

  void TickRender();

  virtual ~Context() = default;

  Window& GetWindow();

private:
  std::vector<utils::uptr<RenderBase>> renderers_;
  utils::uptr<Impl> impl_;
  Window window_;
};

Context& get_context();

}  // namespace ax::gui
