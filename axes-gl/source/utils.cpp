//
// Created by Yang Jerry on 2024/3/3.
//
#include "axes/gl/utils.hpp"

#include <axes/gl/context.hpp>

#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/utils/status.hpp"

namespace ax::gl {

void init() {
  ax::add_init_hook("InitializeAxglContext", []() {
    auto & c = add_resource<Context>();
    AX_RETURN_OK();
  });

  ax::add_clean_up_hook("CleanupAxglContext", []() {
    erase_resource<Context>();
    AX_RETURN_OK();
  });
}

void init(int argc, char** argv) {
  init();
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