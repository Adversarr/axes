//
// Created by Yang Jerry on 2024/3/3.
//
#include "ax/gl/utils.hpp"

#include <ax/gl/context.hpp>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"

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
  init(true);
  get_program_options().add_options()("gl_hidpi_scale", "HiDPI scale factor",
                                      cxxopts::value<float>()->default_value("1.5"));
  ax::init(argc, argv);
}

void enter_main_loop() {
  auto& c = get_resource<Context>();
  auto& w = c.GetWindow();
  while (!(c.ShouldClose() || w.ShouldClose())) {
    c.TickLogic();
    c.TickRender();
  }
}

}  // namespace ax::gl
