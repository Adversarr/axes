#include "gl/window.hpp"

#include <entt/signal/emitter.hpp>
#include <entt/signal/sigh.hpp>

#include "axes/core/entt.hpp"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "axes/core/echo.hpp"
#include "gl/config.hpp"

namespace ax::gl {

/****************************** PImpl ******************************/
struct Window::Impl {
  math::vec2i size_;
  math::vec2i pos_;
  math::vec2i fb_size_;
  math::vec2f fb_scale_;
  bool should_close_;
  GLFWwindow* window_ = nullptr;

  static Window::Impl* Extract(GLFWwindow* window) {
    return static_cast<Window::Impl*>(glfwGetWindowUserPointer(window));
  }
};

/****************************** Callbacks ******************************/
static void window_size_fn(GLFWwindow* window, int width, int height) {
  WindowSizeEvent event;
  event.size_ = {width, height};

  auto impl = Window::Impl::Extract(window);
  impl->size_ = event.size_;
  emit(event);
}

static void window_pos_fn(GLFWwindow* window, int pos_x, int pos_y) {
  WindowPosEvent event;
  event.pos_ = {pos_x, pos_y};

  auto impl = Window::Impl::Extract(window);
  impl->pos_ = event.pos_;
  emit(event);
}

static void framebuffer_size_fn(GLFWwindow* window, int width, int height) {
  FrameBufferSizeEvent event;
  event.size_ = {width, height};

  auto impl = Window::Impl::Extract(window);
  impl->fb_size_ = event.size_;
  emit(event);
}
static void drop_fn(GLFWwindow* /* window */, int count, const char** paths) {
  DropEvent event;
  for (int i = 0; i < count; ++i) {
    event.paths_.emplace_back(paths[i]);
  }
  emit(event);
}

static void key_fn(GLFWwindow* /* window */, int key, int scancode, int action, int mods) {
  KeyboardEvent event;
  event.key_ = key;
  event.scancode_ = scancode;
  event.action_ = action;
  event.mods_ = mods;
  emit(event);
}

static void cursor_pos_fn(GLFWwindow* /* window */, double pos_x, double pos_y) {
  CursorMove event;
  event.pos_ = {pos_x, pos_y};
  emit(event);
}

static void scroll_fn(GLFWwindow* /* window */, double offset_x, double offset_y) {
  ScrollEvent event;
  event.offset_ = {offset_x, offset_y};
  emit(event);
}

static void mouse_button_fn(GLFWwindow* /* window */, int button, int action, int mods) {
  MouseButtonEvent event;
  event.button_ = button;
  event.action_ = action;
  event.mods_ = mods;
  emit(event);
}

Window::Window() {
  impl_ = std::make_unique<Impl>();
  CHECK(glfwInit()) << "Failed to initialize GLFW";

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, AXGL_MAJOR_VERSION);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, AXGL_MINOR_VERSION);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef AX_PLATFORM_APPLE
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  impl_->window_ = glfwCreateWindow(1280, 720, "Axes", nullptr, nullptr);
  CHECK(impl_->window_ != nullptr) << "Failed to create GLFW window";

  glfwMakeContextCurrent(impl_->window_);
  glfwSwapInterval(1);

  /****************************** Initialize internals ******************************/
  int width, height;
  glfwGetWindowSize(impl_->window_, &width, &height);
  impl_->size_ = {width, height};
  glfwGetWindowPos(impl_->window_, &width, &height);
  impl_->pos_ = {width, height};
  glfwGetFramebufferSize(impl_->window_, &width, &height);
  impl_->fb_size_ = {width, height};
  glfwGetWindowContentScale(impl_->window_, &impl_->fb_scale_.x(), &impl_->fb_scale_.y());
  impl_->should_close_ = false;
  glfwSwapInterval(1);  // Enable vsync

  /****************************** Install Fn ******************************/
  glfwSetWindowSizeCallback(impl_->window_, window_size_fn);
  glfwSetWindowPosCallback(impl_->window_, window_pos_fn);
  glfwSetFramebufferSizeCallback(impl_->window_, framebuffer_size_fn);
  glfwSetDropCallback(impl_->window_, drop_fn);
  glfwSetKeyCallback(impl_->window_, key_fn);
  glfwSetCursorPosCallback(impl_->window_, cursor_pos_fn);
  glfwSetScrollCallback(impl_->window_, scroll_fn);
  glfwSetMouseButtonCallback(impl_->window_, mouse_button_fn);

  /****************************** Install User Pointer ******************************/
  glfwSetWindowUserPointer(impl_->window_, impl_.get());
  LOG(INFO) << "Window created";
}

Window::~Window() {
  if (impl_ != nullptr && impl_->window_ != nullptr) {
    glfwDestroyWindow(impl_->window_);
  }
  glfwTerminate();
  LOG(INFO) << "Window destroyed";
}

/****************************** Meta Data Getters ******************************/
math::vec2i Window::GetSize() const { return impl_->size_; }

math::vec2i Window::GetPos() const { return impl_->pos_; }

math::vec2i Window::GetFrameBufferSize() const { return impl_->fb_size_; }

math::vec2r Window::GetCursorPos() const {
  double pos_x, pos_y;
  glfwGetCursorPos(impl_->window_, &pos_x, &pos_y);
  return {pos_x, pos_y};
}

void* Window::GetWindowInternal() const { return impl_->window_; }

}  // namespace ax::gl
