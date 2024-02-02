#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "gl/context.hpp"
#include "gl/program.hpp"
#include "gl/shader.hpp"
#include "gl/window.hpp"

ABSL_FLAG(bool, echo, false, "Echo events");

using namespace ax;

struct Echo {
  void WSize(const gl::WindowSizeEvent& evt) const {
    LOG(INFO) << "Window size: " << math::transpose(evt.size_);
  }

  void WPos(const gl::WindowPosEvent& evt) const {
    LOG(INFO) << "Window pos: " << math::transpose(evt.pos_);
  }

  void Key(const gl::KeyboardEvent& evt) const {
    LOG(INFO) << "Key: " << evt.key_ << std::endl << "Action: " << evt.action_;
  }
};

const char* vertexShaderSource
    = "#version 410 core\n"
      "layout (location = 0) in vec3 aPos;\n"
      "out vec3 position;"
      "void main()\n"
      "{\n"
      "   gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
      "position = aPos;"
      "}\0";
const char* fragmentShaderSource
    = "#version 410 core\n"
      "out vec4 FragColor;\n"
      "in vec3 position;"
      "void main()\n"
      "{\n"
      "   FragColor = vec4(position.xy, 0.2f, 1.0f);\n"
      "}\n\0";

float vertices[] = {0.5f, 0.5f, 0.0f, 0.5f, -0.5f, 0.0f, -0.5f, -0.5f, 0.0f, -0.5f, 0.5f, 0.0f};

unsigned int indices[] = {0, 1, 3, 1, 2, 3};

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& win = add_resource<gl::Window>();
  add_resource<gl::Context>();

  auto vert = gl::Shader::Compile(vertexShaderSource, ax::gl::ShaderType::kVertex);
  auto frag = gl::Shader::Compile(fragmentShaderSource, ax::gl::ShaderType::kFragment);
  CHECK_OK(vert);
  CHECK_OK(frag);

  auto prog = std::make_unique<gl::Program>();
  auto link_status = prog->Append(std::move(vert.value())).Append(std::move(frag.value())).Link();
  CHECK_OK(link_status);

  if (absl::GetFlag(FLAGS_echo)) {
    auto& echo = add_resource<Echo>();
    connect<gl::WindowSizeEvent, &Echo::WSize>(echo);
    connect<gl::WindowPosEvent, &Echo::WPos>(echo);
    connect<gl::KeyboardEvent, &Echo::Key>(echo);
  }

  unsigned int VBO, VAO, EBO;
  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);
  glGenBuffers(1, &EBO);
  glBindVertexArray(VAO);
  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  GLFWwindow* window = static_cast<GLFWwindow*>(win.GetWindowInternal());
  while (!glfwWindowShouldClose(window)) {
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    CHECK_OK(prog->Use());
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    auto win_size = win.GetSize();
    glViewport(0, 0, win_size[0], win_size[1]);

    glfwPollEvents();
    glfwSwapBuffers(window);
  }

  // NOTE: Add to hooks.
  prog.reset();
  erase_resource<gl::Window>();
  erase_resource<gl::Context>();
  clean_up();
  return 0;
}
