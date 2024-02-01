#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "gui/program.hpp"
#include "gui/context.hpp"
#include "gui/shader.hpp"
#include "gui/window.hpp"

ABSL_FLAG(bool, echo, false, "Echo events");

using namespace ax;

struct Echo {
  void WSize(const gui::WindowSizeEvent& evt) const {
    LOG(INFO) << "Window size: " << math::transpose(evt.size_);
  }

  void WPos(const gui::WindowPosEvent& evt) const {
    LOG(INFO) << "Window pos: " << math::transpose(evt.pos_);
  }

  void Key(const gui::KeyboardEvent& evt) const {
    LOG(INFO) << "Key: " << evt.key_ << std::endl << "Action: " << evt.action_;
  }
};

const char* vertexShaderSource
    = "#version 330 core\n"
      "layout (location = 0) in vec3 aPos;\n"
      "void main()\n"
      "{\n"
      "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
      "}\0";
const char* fragmentShaderSource
    = "#version 330 core\n"
      "out vec4 FragColor;\n"
      "void main()\n"
      "{\n"
      "   FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
      "}\n\0";
float vertices[] = {
    0.5f, 0.5f, 0.0f,
    0.5f, -0.5f, 0.0f,
    -0.5f, -0.5f, 0.0f,
    -0.5f, 0.5f, 0.0f
};

unsigned int indices[] = {
    0, 1, 3,
    1, 2, 3
};

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& win = add_resource<gui::Window>();
  auto& ctx = add_resource<gui::Context>();

  auto vert = gui::Shader::Compile(vertexShaderSource, ax::gui::ShaderType::kVertex);
  auto frag = gui::Shader::Compile(fragmentShaderSource, ax::gui::ShaderType::kFragment);
  CHECK_OK(vert);
  CHECK_OK(frag);

  gui::Program prog;
  auto link_status = prog.Append(std::move(vert.value()))
      .Append(std::move(frag.value()))
      .Link();
  CHECK_OK(link_status);

  if (absl::GetFlag(FLAGS_echo)) {
    auto& echo = add_resource<Echo>();
    connect<gui::WindowSizeEvent, &Echo::WSize>(echo);
    connect<gui::WindowPosEvent, &Echo::WPos>(echo);
    connect<gui::KeyboardEvent, &Echo::Key>(echo);
  }

  GLFWwindow* window = static_cast<GLFWwindow*>(win.GetWindowInternal());
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    glfwSwapBuffers(window);
  }

  // NOTE: Add to hooks.
  erase_resource<gui::Window>();
  erase_resource<gui::Context>();
  clean_up();
  return 0;
}
