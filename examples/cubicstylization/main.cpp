#include <imgui.h>

#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/status.hpp"

using namespace ax;

void ui_callback(gl::UiRenderEvent) { ImGui::ShowDemoWindow(); }

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}