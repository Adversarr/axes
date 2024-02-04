#include "axes/gl/context.hpp"

#include "axes/core/entt.hpp"
#include "axes/utils/status.hpp"
#include "impl/render_mesh.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "axes/core/echo.hpp"
#include "axes/gl/config.hpp"
#include "axes/gl/details/gl_call.hpp"
#include "impl/render_line.hpp"

namespace ax::gl {

struct Context::Impl {
  std::vector<utils::uptr<RenderBase>> renderers_;

  Window window_;
  Camera camera_;
  Light light_;

  math::mat4r model_;

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

  void OnFramebufferSize(const FrameBufferSizeEvent& evt) {
    glViewport(0, 0, evt.size_.x(), evt.size_.y());
    camera_.SetAspect(evt.size_.x(), evt.size_.y());
  }
};

Context::Context(Context&&) noexcept = default;

Context::Context() {
  impl_ = std::make_unique<Impl>();
  int status = gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));
  CHECK(status) << "Failed to initialize OpenGL context";
  LOG(INFO) << "Setup OpenGL context";

  connect<KeyboardEvent, &Impl::OnKey>(*impl_);
  connect<FrameBufferSizeEvent, &Impl::OnFramebufferSize>(*impl_);

  auto fb_size = impl_->window_.GetFrameBufferSize();
  impl_->camera_.SetAspect(fb_size.x(), fb_size.y());
  impl_->model_ = math::eye<4>();
  impl_->light_.position_ = impl_->camera_.GetPosition();
  impl_->light_.ambient_strength_ = 0.1;

  /****************************** Setup ImGui ******************************/
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)impl_->window_.GetWindowInternal(), true);
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCanvasResizeCallback("#canvas");
#endif
  ImGui_ImplOpenGL3_Init(AXGL_GLSL_VERSION_TAG);

  auto fb_scale = impl_->window_.GetFrameBufferScale();
  ImGui::GetIO().DisplayFramebufferScale.x = fb_scale.x();
  ImGui::GetIO().DisplayFramebufferScale.y = fb_scale.y();

  /****************************** Setup Subrenderers ******************************/
  impl_->renderers_.emplace_back(std::make_unique<LineRenderer>());
  impl_->renderers_.emplace_back(std::make_unique<MeshRenderer>());
  for (auto& renderer : impl_->renderers_) {
    CHECK_OK(renderer->Setup());
  }
}

Context::~Context() {
  global_dispatcher().clear<UiRenderEvent>();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  impl_->renderers_.clear();
  impl_.reset();
  LOG(INFO) << "Destroy OpenGL context";
}

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
  glDisable(GL_CULL_FACE);
  glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (auto& renderer : impl_->renderers_) {
    AX_EVAL_RETURN_NOTOK(renderer->TickRender());
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  emit<UiRenderEvent>({});
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  AXGL_CALLR(glFlush());
  impl_->window_.SwapBuffers();
  AX_RETURN_OK();
}

Light& Context::GetLight() { return impl_->light_; }

math::mat4r const& Context::GetGlobalModelMatrix() const { return impl_->model_; }

void Context::SetGlobalModelMatrix(math::mat4r const& value) { impl_->model_ = value; }

}  // namespace ax::gl
