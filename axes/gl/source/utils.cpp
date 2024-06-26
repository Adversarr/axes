//
// Created by Yang Jerry on 2024/3/3.
//
#include "ax/gl/utils.hpp"

#include <ax/gl/context.hpp>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/utils/status.hpp"

namespace ax::gl {

void init() {
  ax::add_init_hook("InitializeAxglContext", []() {
    add_resource<Context>();
    AX_RETURN_OK();
  });

  ax::add_clean_up_hook("CleanupAxglContext", []() {
    erase_resource<Context>();
    AX_RETURN_OK();
  });
}

void init(int argc, char** argv) {
  gl::init();
  ::ax::init(argc, argv);
}

Status enter_main_loop() {
  auto & c = get_resource<Context>();
  while (!c.GetWindow().ShouldClose()) {
    AX_RETURN_NOTOK(c.TickLogic());
    AX_RETURN_NOTOK(c.TickRender());
  }
  AX_RETURN_OK();
}

}