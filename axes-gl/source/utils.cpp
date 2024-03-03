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
  ax::hook_init("InitializeAxglContext", []() {
    add_resource<Context>();
    AX_RETURN_OK();
  });

  ax::hook_clean_up("CleanupAxglContext", []() {
    erase_resource<Context>();
    AX_RETURN_OK();
  });
}

void init(int argc, char** argv) {
  init();
  ::ax::init(argc, argv);
}


}