#include "gl/context.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "axes/core/echo.hpp"

namespace ax::gl {

struct Context::Impl {};

Context::Context() {
  impl_ = std::make_unique<Impl>();
  int status = gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));
  CHECK(status) << "Failed to initialize OpenGL context";
  LOG(INFO) << "Setup OpenGL context";
}

Context::~Context() {
  LOG(INFO) << "Destroy OpenGL context";
}

}  // namespace ax::gl
