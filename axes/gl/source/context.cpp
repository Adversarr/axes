#include "ax/gl/context.hpp"

#include <implot.h>

#include "ax/components/name.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/excepts.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/extprim/axes.hpp"
#include "ax/gl/extprim/grid.hpp"
#include "impl/render_line.hpp"
#include "impl/render_mesh.hpp"
#include "impl/render_point.hpp"
#include "impl/render_quiver.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/gl/config.hpp"
#include "ax/gl/details/gl_call.hpp"
#include "ax/math/utils/structure_binding.hpp"
#include "ax/utils/asset.hpp"

namespace ax::gl {

using math::cast;

struct Context::Impl {
  bool has_setuped_{false};
  bool acknowledged_should_shutdown_{false};
  std::vector<std::unique_ptr<RenderBase>> renderers_;

  Window window_;
  Camera camera_;
  Light light_;

  f32 model_rotate_x_{0.0f};
  f32 model_rotate_y_{0.0f};

  math::FloatMatrix4 model_;
  math::FloatVector4 clear_color_;

  entt::entity axis_entity_;
  entt::entity light_entity_;
  entt::entity grid_entity_;

  bool render_axis_{true};
  bool render_light_{false};
  bool render_grid_{true};

  bool update_light_{true};
  bool update_axes_{true};
  bool update_grid_{true};

  bool is_pressing_meta_key_ = false;
  bool is_pressing_ctrl_key_ = false;
  bool is_pressing_shft_key_ = false;
  bool is_pressing_space_key_ = false;
  bool is_mouse_button_pressed_ = false;
  bool is_context_window_open_{false};
  math::RealVector2 prev_cursor_pos_;
  float mouse_sensitivity_{1.0f};

  std::vector<entt::connection> connections_;

  void OnContextShouldShutdown(ContextShouldShutdownEvent const&) { acknowledged_should_shutdown_ = true; }

  void OnKey(const KeyboardEvent& evt);

  void OnFramebufferSize(const FrameBufferSizeEvent& evt);

  void OnCursorMove(const CursorMoveEvent& evt);

  void OnMouse(const MouseButtonEvent& evt);

  void OnUiRender(UiRenderEvent const&);

  void UpdateLight();

  void UpdateAxes();

  void UpdateGrid();

  void OnMenuBar() {
    if (ImGui::BeginMenu("File")) {
      ImGui::Checkbox("Settings", &is_context_window_open_);
      ImGui::EndMenu();
    }
  }
};

void Context::Impl::OnKey(const KeyboardEvent& evt) {
  // Test if any imgui is focused
  if (ImGui::GetIO().WantCaptureKeyboard) {
    return;
  }

  if (evt.action_ != GLFW_PRESS && evt.action_ != GLFW_REPEAT) {
    if (evt.key_ == GLFW_KEY_LEFT_SHIFT || evt.key_ == GLFW_KEY_RIGHT_SHIFT) {
      is_pressing_shft_key_ = false;
    } else if (evt.key_ == GLFW_KEY_LEFT_CONTROL || evt.key_ == GLFW_KEY_RIGHT_CONTROL) {
      is_pressing_ctrl_key_ = false;
    } else if (evt.key_ == GLFW_KEY_LEFT_ALT || evt.key_ == GLFW_KEY_RIGHT_ALT) {
      is_pressing_meta_key_ = false;
    } else if (evt.key_ == GLFW_KEY_SPACE) {
      is_pressing_space_key_ = false;
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
  } else if (evt.key_ == GLFW_KEY_SPACE) {
    is_pressing_space_key_ = true;
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

void Context::Impl::OnFramebufferSize(const FrameBufferSizeEvent& evt) {
  glViewport(0, 0, evt.width_, evt.height_);
  camera_.SetAspect(evt.width_, evt.height_);
}

void Context::Impl::OnCursorMove(const CursorMoveEvent& evt) {
  if (ImGui::GetIO().WantCaptureMouse) {
    return;
  }

  if (is_mouse_button_pressed_) {
    float dx = static_cast<float>(evt.pos_.x() - prev_cursor_pos_.x());
    float dy = static_cast<float>(prev_cursor_pos_.y() - evt.pos_.y());

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

    if (is_pressing_space_key_) {
      // Rotate the world model matrix.
      dx *= mouse_sensitivity_ * 0.005f;
      dy *= mouse_sensitivity_ * 0.005f;
      model_rotate_x_ += dx;
      model_rotate_y_ += dy;

      // Rotate the camera about zero.
      auto up = math::FloatVector3::UnitY();
      auto right = camera_.GetRight();
      const math::FloatMatrix3 rx = Eigen::AngleAxis<f32>(dy, right).toRotationMatrix();
      const math::FloatMatrix3 ry = Eigen::AngleAxis<f32>(-dx, up).toRotationMatrix();
      const math::FloatMatrix3 rot = rx * ry;
      math::FloatMatrix4 affine = math::eye<4, float>();
      affine.topLeftCorner<3, 3>() = rot;
      camera_.ApplyTransform(affine);
    }
  }
  prev_cursor_pos_ = evt.pos_;
}

void Context::Impl::OnMouse(const MouseButtonEvent& evt) {
  if (evt.action_ == GLFW_PRESS) {
    is_mouse_button_pressed_ = true;
    // prev_cursor_pos_ = window_.GetCursorPos();
  } else if (evt.action_ == GLFW_RELEASE) {
    is_mouse_button_pressed_ = false;
  }
}

void Context::Impl::OnUiRender(UiRenderEvent const&) {
  if (!is_context_window_open_) {
    return;
  }
  if (ImGui::Begin("AXGL Context", &is_context_window_open_)) {
    if (ImGui::CollapsingHeader("Scene")) {
      ImGui::InputFloat("Mouse Sensitivity", &mouse_sensitivity_);
      ImGui::Text("Camera Position: %.2f, %.2f, %.2f", camera_.GetPosition().x(), camera_.GetPosition().y(),
                  camera_.GetPosition().z());
      ImGui::Text("Camera Yaw=%.2f Pitch=%.2f", camera_.GetYaw(), camera_.GetPitch());
      ImGui::Checkbox("Perspective", &camera_.use_perspective_);
      update_axes_ = ImGui::Checkbox("Render axes", &render_axis_);
      update_grid_ = ImGui::Checkbox("Render grid", &render_grid_);
      update_light_ = ImGui::Checkbox("Render light", &render_light_);
      update_light_ |= ImGui::InputFloat3("Light", &light_.position_.x());
      update_light_ |= ImGui::SliderFloat("Light Ambi", &light_.ambient_strength_, 0.0f, 1.0f);
      update_light_ |= ImGui::SliderFloat("Light Diff", &light_.diffuse_strength_, 0.0f, 1.0f);
      update_light_ |= ImGui::SliderFloat("Light Spec", &light_.specular_strength_, 0.0f, 1.0f);
      ImGui::ColorEdit3("Clear Color", &clear_color_.x());
    }

    if (ImGui::CollapsingHeader("Help & Shortcuts")) {
      ImGui::BulletText("Use HJKL to rotate camera");
      ImGui::BulletText("Use WASD to move camera");
      ImGui::BulletText("Use Alt+Mouse to rotate camera");
      ImGui::BulletText("Use Shift+Mouse to move camera");
      ImGui::BulletText("Use Ctrl+Mouse to zoom (also move) camera");
      ImGui::BulletText("Use Space+Mouse to rotate the whole world");
    }
    if (ImGui::CollapsingHeader("Renderers")) {
      for (auto& renderer : renderers_) {
        renderer->RenderGui();
      }
    }
  }
  ImGui::End();
}

void Context::Impl::UpdateLight() {
  if (!update_light_) {
    return;
  }
  if (render_light_) {
    if (!has_component<Mesh>(light_entity_)) {
      add_component<Mesh>(light_entity_);
    }
    patch_component<Mesh>(light_entity_, [&](Mesh& mesh) {
      auto const cube = geo::cube(0.03);
      mesh.vertices_ = cube.vertices_;
      mesh.indices_ = cube.indices_;
      math::each(mesh.vertices_) += light_.position_.cast<f64>();
      mesh.colors_ = math::ones<4>(mesh.vertices_.cols());
      mesh.use_lighting_ = false;
      mesh.is_flat_ = true;
      mesh.use_global_model_ = true;
    });
  } else {
    remove_component<Mesh>(light_entity_);
  }
  update_light_ = false;
}

void Context::Impl::UpdateAxes() {
  if (!update_axes_) {
    return;
  }
  if (render_axis_) {
    if (!has_component<Lines>(axis_entity_)) {
      add_component<Lines>(axis_entity_);
    }
    patch_component<Lines>(axis_entity_, [&](Lines& lines) {
      lines = prim::Axes().Draw();
      lines.dim_far_away_from_center_ = false;
    });
  } else {
    remove_component<Lines>(axis_entity_);
  }
  update_axes_ = false;
}
void Context::Impl::UpdateGrid() {
  if (!update_grid_) {
    return;
  }
  if (render_grid_) {
    if (!has_component<Lines>(grid_entity_)) {
      add_component<Lines>(grid_entity_);
    }
    patch_component<Lines>(grid_entity_, [&](Lines& lines) {
      lines = prim::Grid().Draw();
      lines.colors_.setConstant(0.3);
      lines.dim_far_away_from_center_ = false;
    });
  } else {
    remove_component<Lines>(grid_entity_);
  }
  update_grid_ = false;
}

Context::Context(Context&&) noexcept = default;

float get_hidpi_scale() {
  auto scale = 1.0f;
  auto const& prog_option = get_parse_result();
  for (auto const& option : prog_option) {
    if (option.key() == "gl_hidpi_scale") {
      scale = prog_option["gl_hidpi_scale"].as<float>();
      AX_TRACE("Setting HiDPI scale to {}", scale);
      break;
    }
  }
  return scale;
}

Context::Context() {
  impl_ = std::make_unique<Impl>();
  AX_THROW_IF_FALSE(gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)),
                    "Failed to initialize OpenGL context");

  auto fb_size = impl_->window_.GetFrameBufferSize();
  impl_->camera_.SetAspect(fb_size.x(), fb_size.y());
  impl_->model_ = math::eye<4>().cast<float>();
  impl_->clear_color_ = math::FloatVector4{0.1f, 0.1f, 0.1f, 1.0f};
  impl_->light_.position_ = math::FloatVector3{0, 2, 2};
  impl_->light_.ambient_strength_ = 0.1f;
  impl_->prev_cursor_pos_ = impl_->window_.GetCursorPos();

  impl_->axis_entity_ = cmpt::create_named_entity("SceneAxis");
  impl_->light_entity_ = cmpt::create_named_entity("SceneLight");
  impl_->grid_entity_ = cmpt::create_named_entity("SceneGrid");

  /* SECT: std::vectoren on Signals */
  impl_->connections_.emplace_back(connect<ContextShouldShutdownEvent, &Impl::OnContextShouldShutdown>(*impl_));
  impl_->connections_.emplace_back(connect<KeyboardEvent, &Impl::OnKey>(*impl_));
  impl_->connections_.emplace_back(connect<FrameBufferSizeEvent, &Impl::OnFramebufferSize>(*impl_));
  impl_->connections_.emplace_back(connect<CursorMoveEvent, &Impl::OnCursorMove>(*impl_));
  impl_->connections_.emplace_back(connect<MouseButtonEvent, &Impl::OnMouse>(*impl_));
  impl_->connections_.emplace_back(connect<UiRenderEvent, &Impl::OnUiRender>(*impl_));

  /* SECT: Setup ImGUI */
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

  ImGui::StyleColorsDark();
  float scale = get_hidpi_scale();
  ImGui::GetStyle().ScaleAllSizes(scale);
  ImGui::GetIO().FontGlobalScale = scale;

  AX_THROW_IF_FALSE(ImGui_ImplGlfw_InitForOpenGL(static_cast<GLFWwindow*>(impl_->window_.GetWindowInternal()), true),
                    "Failed to Initialize ImGUI_Glfw");
#ifdef __EMSCRIPTEN__
  ImGui_ImplGlfw_InstallEmscriptenCanvasResizeCallback("#canvas");
#endif
  AX_THROW_IF_FALSE(ImGui_ImplOpenGL3_Init(AXGL_GLSL_VERSION_TAG), "Failed to Initialize ImGUI_OpenGL3");

  /* SECT: Setup SubRenderers */
  impl_->renderers_.emplace_back(std::make_unique<LineRenderer>());
  impl_->renderers_.emplace_back(std::make_unique<MeshRenderer>());
  impl_->renderers_.emplace_back(std::make_unique<PointRenderer>());
  impl_->renderers_.emplace_back(std::make_unique<QuiverRenderer>());
}

Context::~Context() {
  for (auto& c : impl_->connections_) {
    c.release();
  }

  emit<ContextDestroyEvent>({});  // Emit the destroy event
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  impl_->renderers_.clear();
  impl_.reset();
  AX_TRACE("Destroy OpenGL context");
}

Window& Context::GetWindow() { return impl_->window_; }

Camera& Context::GetCamera() { return impl_->camera_; }

/************************* SECT: Tick Logic and Tick Renderers *************************/

void Context::Initialize() {
  if (impl_->has_setuped_) {
    return;
  }
  for (auto& renderer : impl_->renderers_) {
    renderer->Setup();
  }
  emit(ContextInitEvent{});
  impl_->has_setuped_ = true;
}

void Context::TickLogic() {
  if_unlikely(!impl_->has_setuped_) { Initialize(); }
  impl_->window_.PollEvents();
  impl_->UpdateLight();
  impl_->UpdateAxes();
  impl_->UpdateGrid();
  for (auto& renderer : impl_->renderers_) {
    renderer->TickLogic();
  }
  emit_enqueue(tick_logic_event);
  trigger_queue<TickLogicEvent>();
}

void Context::TickRender() {
  // SECT: Setup OpenGL context
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  auto [r, g, b, a] = math::unpack(impl_->clear_color_);
  glClearColor(r, g, b ,a);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  for (auto& renderer : impl_->renderers_) {
    renderer->TickRender();
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Must be called before any ImGui window call.
  ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);
  emit<UiRenderEvent>({});

  if (ImGui::BeginMainMenuBar()) {
    impl_->OnMenuBar();
    emit<MainMenuBarRenderEvent>({});
    ImGui::EndMainMenuBar();
  }

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable) {
    GLFWwindow* backup_current_context = glfwGetCurrentContext();
    ImGui::UpdatePlatformWindows();
    ImGui::RenderPlatformWindowsDefault();
    glfwMakeContextCurrent(backup_current_context);
  }

  AXGL_CALL(glFlush());
  impl_->window_.SwapBuffers();
}

bool Context::ShouldClose() const { return impl_->acknowledged_should_shutdown_; }

void Context::AppendEntityRenderer(std::unique_ptr<RenderBase> renderer) {
  impl_->renderers_.push_back(std::move(renderer));
}

Light& Context::GetLight() { return impl_->light_; }

math::FloatMatrix4 const& Context::GetGlobalModelMatrix() const { return impl_->model_; }

void Context::SetGlobalModelMatrix(math::FloatMatrix4 const& value) { impl_->model_ = value; }

}  // namespace ax::gl
