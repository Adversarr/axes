#pragma once

#include <vector>

#include "axes/utils/common.hpp"
#include "render_base.hpp"
namespace ax::gui {

class Context {
public:
  struct Impl;
  Context();

  void TickRender();

  ~Context();

private:
  std::vector<utils::uptr<RenderBase>> renderers_;
  utils::uptr<Impl> impl_;
};

Context& get_context();

}  // namespace ax::gui
