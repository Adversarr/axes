//
// Created by Yang Jerry on 2024/3/3.
//
#include "ax/gl/utils.hpp"

#include <ax/gl/context.hpp>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/utils/status.hpp"

namespace ax::gl {

void init(bool is_registering) {
  if (is_registering) {
    ax::add_init_hook("InitializeAxglContext", []() -> void { add_resource<Context>(); });
  } else {
    add_resource<Context>();
  }

  ax::add_clean_up_hook("CleanupAxglContext", []() -> void { erase_resource<Context>(); });
}

void init(int argc, char** argv) {
  gl::init(true);
  ::ax::init(argc, argv);
}

Status enter_main_loop() {
  auto& c = get_resource<Context>();
  auto& w = c.GetWindow();
  while (!(c.ShouldClose() || w.ShouldClose())) {
    AX_RETURN_NOTOK(c.TickLogic());
    AX_RETURN_NOTOK(c.TickRender());
  }
  AX_RETURN_OK();
}

}  // namespace ax::gl
