#include "axes/gl/context.hpp"

#include "axes/core/entt.hpp"
#include "axes/utils/status.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "axes/core/echo.hpp"
#include "axes/gl/details/gl_call.hpp"
#include "impl/render_line.hpp"

namespace ax::gl {

struct FramebufferResizeHandler {
  void OnEvent(const FrameBufferSizeEvent& evt) const { glViewport(0, 0, evt.size_.x(), evt.size_.y()); }
};

struct Context::Impl {
  std::vector<utils::uptr<RenderBase>> renderers_;

  Window window_;
  Camera camera_;
  FramebufferResizeHandler resize_event_handler_;

  void OnKey(const KeyboardEvent& evt) {
    if (evt.action_ != GLFW_PRESS && evt.action_ != GLFW_REPEAT) {
      return;
    }
    if (evt.key_ == GLFW_KEY_H) {
      camera_.Rotate(-5.0f, 0.0f);
    } else if (evt.key_ == GLFW_KEY_L) {
      camera_.Rotate(5.0f, 0.0f);
    } else if (evt.key_ == GLFW_KEY_J) {
      camera_.Rotate(0.0f, -5.0f);
    } else if (evt.key_ == GLFW_KEY_K) {
      camera_.Rotate(0.0f, 5.0f);
    } else if (evt.key_ == GLFW_KEY_W) {
      camera_.Move(camera_.GetFront() * 0.1f);
    } else if (evt.key_ == GLFW_KEY_S) {
      camera_.Move(-camera_.GetFront() * 0.1f);
    } else if (evt.key_ == GLFW_KEY_A) {
      camera_.Move(-camera_.GetRight() * 0.1f);
    } else if (evt.key_ == GLFW_KEY_D) {
      camera_.Move(camera_.GetRight() * 0.1f);
    } else if (evt.key_ == GLFW_KEY_Q) {
      camera_.Move(camera_.GetUp() * 0.1f);
    } else if (evt.key_ == GLFW_KEY_E) {
      camera_.Move(-camera_.GetUp() * 0.1f);
    }
  }
};

Context::Context(Context&&) noexcept = default;

Context::Context() {
  impl_ = std::make_unique<Impl>();
  int status = gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));
  CHECK(status) << "Failed to initialize OpenGL context";
  LOG(INFO) << "Setup OpenGL context";

  connect<FrameBufferSizeEvent, &FramebufferResizeHandler::OnEvent>(impl_->resize_event_handler_);
  connect<KeyboardEvent, &Impl::OnKey>(impl_.get());

  impl_->renderers_.emplace_back(std::make_unique<LineRenderer>());

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)impl_->window_.GetWindowInternal(), true);
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCanvasResizeCallback("#canvas");
#endif
  ImGui_ImplOpenGL3_Init("#version 410");

  for (auto& renderer : impl_->renderers_) {
    CHECK_OK(renderer->Setup());
  }
}

Context::~Context() { LOG(INFO) << "Destroy OpenGL context"; }

Window& Context::GetWindow() { return impl_->window_; }

Camera& Context::GetCamera() { return impl_->camera_; }

Status Context::TickLogic() {
  impl_->window_.PollEvents();
  for (auto& renderer : impl_->renderers_) {
    AX_EVAL_RETURN_NOTOK(renderer->TickLogic());
  }
  AX_RETURN_OK();
}

Status Context::TickRender() {
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (auto& renderer : impl_->renderers_) {
    AX_EVAL_RETURN_NOTOK(renderer->TickRender());
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  ImGui::ShowDemoWindow();

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  AXGL_CALLR(glFlush());
  impl_->window_.SwapBuffers();
  AX_RETURN_OK();
}

}  // namespace ax::gl
