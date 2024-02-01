#include "gui/context.hpp"
#include "axes/core/echo.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

namespace ax::gui {

struct Context::Impl {

};

Context::Context() {
  // LOG(INFO) << "Setup OpenGL context";
  // impl_ = std::make_unique<Impl>();
  // gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));
  // TODO: Initialize it
}

}  // namespace ax::gui
