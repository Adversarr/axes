#include "axes/gl/context.hpp"

#include "axes/core/entt.hpp"
#include "axes/utils/status.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "axes/core/echo.hpp"

#include "axes/gl/details/gl_call.hpp"
#include "impl/render_line.hpp"

namespace ax::gl {

struct FramebufferResizeHandler {
  void OnEvent(const FrameBufferSizeEvent& evt) const {
    glViewport(0, 0, evt.size_.x(), evt.size_.y());
  }
};

struct Context::Impl {
  std::vector<utils::uptr<RenderBase>> renderers_;

  Window window_;
  FramebufferResizeHandler resize_event_handler_;
};

Context::Context(Context&&) noexcept = default;

Context::Context() {
  impl_ = std::make_unique<Impl>();
  int status = gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));
  CHECK(status) << "Failed to initialize OpenGL context";
  LOG(INFO) << "Setup OpenGL context";

  connect<FrameBufferSizeEvent, &FramebufferResizeHandler::OnEvent>(impl_->resize_event_handler_);

  impl_->renderers_.emplace_back(std::make_unique<LineRenderer>());

  for (auto& renderer : impl_->renderers_) {
    CHECK_OK(renderer->Setup());
  }
}

Context::~Context() { LOG(INFO) << "Destroy OpenGL context"; }

Window& Context::GetWindow() { return impl_->window_; }

Status Context::TickLogic() {
  impl_->window_.PollEvents();
  for (auto& renderer : impl_->renderers_) {
    AX_EVAL_RETURN_NOTOK(renderer->TickLogic());
  }
  AX_RETURN_OK();
}

Status Context::TickRender() {
  glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (auto& renderer : impl_->renderers_) {
    AX_EVAL_RETURN_NOTOK(renderer->TickRender());
  }
  AXGL_CALLR(glFlush());
  impl_->window_.SwapBuffers();
  AX_RETURN_OK();
}

}  // namespace ax::gl
