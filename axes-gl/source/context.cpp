#include "axes/gl/context.hpp"

#include "axes/core/entt.hpp"
#include "axes/geometry/primitives.hpp"
#include "axes/gl/extprim/axes.hpp"
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

  math::mat4f model_;
  math::vec4f clear_color_;

  entt::entity axis_entity_;
  entt::entity light_entity_;

  bool render_axis_{true};
  bool render_light_{true};

  bool update_light_{true};
  bool update_axes_{true};

  bool is_pressing_meta_key_ = false;
  bool is_pressing_ctrl_key_ = false;
  bool is_pressing_shft_key_ = false;
  bool is_mouse_button_pressed_ = false;
  math::vec2r prev_cursor_pos_;

  float mouse_sensitivity_{1.0f};

  void OnKey(const KeyboardEvent& evt) {
    if (evt.action_ != GLFW_PRESS && evt.action_ != GLFW_REPEAT) {
      if (evt.key_ == GLFW_KEY_LEFT_SHIFT || evt.key_ == GLFW_KEY_RIGHT_SHIFT) {
        is_pressing_shft_key_ = false;
      } else if (evt.key_ == GLFW_KEY_LEFT_CONTROL || evt.key_ == GLFW_KEY_RIGHT_CONTROL) {
        is_pressing_ctrl_key_ = false;
      } else if (evt.key_ == GLFW_KEY_LEFT_ALT || evt.key_ == GLFW_KEY_RIGHT_ALT) {
        is_pressing_meta_key_ = false;
      }
      // NOTE: Nothing is needed.
      return;
    }

    if (evt.key_ == GLFW_KEY_LEFT_SHIFT || evt.key_ == GLFW_KEY_RIGHT_SHIFT) {
      is_pressing_shft_key_ = true;
    } else if (evt.key_ == GLFW_KEY_LEFT_CONTROL || evt.key_ == GLFW_KEY_RIGHT_CONTROL) {
      is_pressing_ctrl_key_ = true;
    } else if (evt.key_ == GLFW_KEY_LEFT_ALT || evt.key_ == GLFW_KEY_RIGHT_ALT) {
      is_pressing_meta_key_ = true;
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

  void OnCursorMove(const CursorMoveEvent& evt) {
    if (is_mouse_button_pressed_) {
      real dx = evt.pos_.x() - prev_cursor_pos_.x();
      real dy = prev_cursor_pos_.y() - evt.pos_.y();

      dx *= mouse_sensitivity_;
      dy *= mouse_sensitivity_;

      if (is_pressing_meta_key_) {
        camera_.Rotate(dx * 0.3f, dy * 0.3f);
      }

      if (is_pressing_shft_key_) {
        auto right = camera_.GetRight();
        auto up = camera_.GetUp();

        camera_.Move(-(up * dy * 0.01f + right * dx * 0.01f));
      }

      if (is_pressing_ctrl_key_) {
        auto front = camera_.GetFront();
        camera_.Move(front * dy * 0.01f);
      }
    }
    prev_cursor_pos_ = evt.pos_;
  }

  void OnMouse(const MouseButtonEvent& evt) {
    if (evt.action_ == GLFW_PRESS) {
      is_mouse_button_pressed_ = true;
      // prev_cursor_pos_ = window_.GetCursorPos();
    } else if (evt.action_ == GLFW_RELEASE) {
      is_mouse_button_pressed_ = false;
    }
  }

  void OnUiRender(UiRenderEvent const&) {
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 100), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("AXGL Context", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
      ImGui::InputFloat("Mouse Sensitivity", &mouse_sensitivity_);
      ImGui::Text("Camera Position: %.2f, %.2f, %.2f", camera_.GetPosition().x(), camera_.GetPosition().y(),
                  camera_.GetPosition().z());
      ImGui::Text("Camera Yaw=%.2f Pitch=%.2f", camera_.GetYaw(), camera_.GetPitch());
      update_axes_ = ImGui::Checkbox("Render axes", &render_axis_);
      update_light_ = ImGui::Checkbox("Render light", &render_light_);
      update_light_ |= ImGui::InputFloat3("Light", &light_.position_.x());
      update_light_ |= ImGui::SliderFloat("Light Ambi", &light_.ambient_strength_, 0.0f, 1.0f);
      update_light_ |= ImGui::SliderFloat("Light Diff", &light_.diffuse_strength_, 0.0f, 1.0f);
      update_light_ |= ImGui::SliderFloat("Light Spec", &light_.specular_strength_, 0.0f, 1.0f);

      ImGui::ColorEdit3("Clear Color", &clear_color_.x());
    }
    ImGui::End();
  }

  void UpdateLight() {
    if (!update_light_) {
      return;
    }
    if (render_light_) {
      auto& mesh = add_or_replace_component<gl::Mesh>(light_entity_);
      std::tie(mesh.vertices_, mesh.indices_) = geo::cube(0.03);
      math::each(mesh.vertices_) += light_.position_.cast<f64>();
      mesh.colors_ = math::ones<4>(mesh.vertices_.cols());
      mesh.use_lighting_ = false;
      mesh.is_flat_ = true;
      mesh.use_global_model_ = false;
      mesh.flush_ = true;
    } else {
      remove_component<gl::Mesh>(light_entity_);
    }
  }

  void UpdateAxes() {
    if (!update_axes_) {
      return;
    }
    if (render_axis_) {
      auto& mesh = add_or_replace_component<gl::Lines>(axis_entity_, gl::prim::Axes().Draw());
      mesh.use_global_model_ = false;
      mesh.flush_ = true;
    } else {
      remove_component<gl::Lines>(axis_entity_);
    }
  }
};

Context::Context(Context&&) noexcept = default;

Context::Context() {
  impl_ = std::make_unique<Impl>();
  int status = gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));
  AX_CHECK(status) << "Failed to initialize OpenGL context";
 AX_LOG(INFO) << "Setup OpenGL context";

  auto fb_size = impl_->window_.GetFrameBufferSize();
  impl_->camera_.SetAspect(fb_size.x(), fb_size.y());
  impl_->model_ = math::eye<4>().cast<float>();
  impl_->clear_color_ = math::vec4f{0.1f, 0.1f, 0.1f, 1.0f};
  impl_->light_.position_ = math::vec3f{0, 2, 2};
  impl_->light_.ambient_strength_ = 0.1f;
  impl_->prev_cursor_pos_ = impl_->window_.GetCursorPos();

  impl_->axis_entity_ = create_entity();
  impl_->light_entity_ = create_entity();

  /* SECT: Listen on Signals */
  connect<KeyboardEvent, &Impl::OnKey>(*impl_);
  connect<FrameBufferSizeEvent, &Impl::OnFramebufferSize>(*impl_);
  connect<CursorMoveEvent, &Impl::OnCursorMove>(*impl_);
  connect<MouseButtonEvent, &Impl::OnMouse>(*impl_);
  connect<UiRenderEvent, &Impl::OnUiRender>(*impl_);

  /* SECT: Setup ImGUI */
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  AX_CHECK(ImGui_ImplGlfw_InitForOpenGL((GLFWwindow*)impl_->window_.GetWindowInternal(), true))
      << "Failed to Initialize ImGUI_GLFW";
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCanvasResizeCallback("#canvas");
#endif
  AX_CHECK(ImGui_ImplOpenGL3_Init(AXGL_GLSL_VERSION_TAG)) << "Failed to Initialize ImGUI_OpenGL3";
  auto fb_scale = impl_->window_.GetFrameBufferScale();
  ImGui::GetIO().DisplayFramebufferScale.x = fb_scale.x();
  ImGui::GetIO().DisplayFramebufferScale.y = fb_scale.y();

  /* SECT: Setup SubRenderers */
  impl_->renderers_.emplace_back(std::make_unique<LineRenderer>());
  impl_->renderers_.emplace_back(std::make_unique<MeshRenderer>());
  for (auto& renderer : impl_->renderers_) {
    AX_CHECK_OK(renderer->Setup());
  }
}

Context::~Context() {
  global_dispatcher().clear<UiRenderEvent>();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  impl_->renderers_.clear();
  impl_.reset();
 AX_LOG(INFO) << "Destroy OpenGL context";
}

Window& Context::GetWindow() { return impl_->window_; }

Camera& Context::GetCamera() { return impl_->camera_; }

/************************* SECT: Tick Logic and Tick Renderers *************************/

Status Context::TickLogic() {
  impl_->window_.PollEvents();
  impl_->UpdateLight();
  impl_->UpdateAxes();
  for (auto& renderer : impl_->renderers_) {
    AX_EVAL_RETURN_NOTOK(renderer->TickLogic());
  }
  AX_RETURN_OK();
}

Status Context::TickRender() {
  // SECT: Setup OpenGL context
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glClearColor(impl_->clear_color_(0), impl_->clear_color_(1), impl_->clear_color_(2),
               impl_->clear_color_(3));
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

math::mat4f const& Context::GetGlobalModelMatrix() const { return impl_->model_; }

void Context::SetGlobalModelMatrix(math::mat4f const& value) { impl_->model_ = value; }

}  // namespace ax::gl
