#include "ax/gl/window.hpp"
#include "ax/gl/context.hpp"

#include <entt/signal/emitter.hpp>
#include <entt/signal/sigh.hpp>

#include "ax/core/entt.hpp"
#include "ax/core/excepts.hpp"
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#include <glad/glad.h>

#include "ax/core/logging.hpp"
#include "ax/math/formatting.hpp"
#include "ax/gl/config.hpp"

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
  emit_enqueue(event);
}

static void window_pos_fn(GLFWwindow* window, int pos_x, int pos_y) {
  WindowPosEvent event;
  event.pos_ = {pos_x, pos_y};

  auto impl = Window::Impl::Extract(window);
  impl->pos_ = event.pos_;
  emit_enqueue(event);
}

static void framebuffer_size_fn(GLFWwindow* window, int width, int height) {
  FrameBufferSizeEvent event;
  event.size_ = {width, height};

  auto impl = Window::Impl::Extract(window);
  impl->fb_size_ = event.size_;
  emit_enqueue(event);
}
static void drop_fn(GLFWwindow* /* window */, int count, const char** paths) {
  DropEvent event;
  for (int i = 0; i < count; ++i) {
    event.paths_.emplace_back(paths[i]);
  }
  emit_enqueue(event);
}

static void key_fn(GLFWwindow* /* window */, int key, int scancode, int action, int mods) {
  KeyboardEvent event;
  event.key_ = key;
  event.scancode_ = scancode;
  event.action_ = action;
  event.mods_ = mods;
  emit_enqueue(event);
}

static void cursor_pos_fn(GLFWwindow* /* window */, double pos_x, double pos_y) {
  CursorMoveEvent event;
  event.pos_ = {pos_x, pos_y};
  emit_enqueue(event);
}

static void scroll_fn(GLFWwindow* /* window */, double offset_x, double offset_y) {
  ScrollEvent event;
  event.offset_ = {offset_x, offset_y};
  emit_enqueue(event);
}

static void mouse_button_fn(GLFWwindow* /* window */, int button, int action, int mods) {
  MouseButtonEvent event;
  event.button_ = button;
  event.action_ = action;
  event.mods_ = mods;
  emit_enqueue(event);
}

Window::Window() {
  impl_ = std::make_unique<Impl>();
  AX_THROW_IF_TRUE(glfwInit() != GLFW_TRUE, "Failed to initialize GLFW");

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, AXGL_MAJOR_VERSION);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, AXGL_MINOR_VERSION);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef AX_PLATFORM_APPLE
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

  float hidpi_scale = get_hidpi_scale();
  impl_->window_ = glfwCreateWindow(static_cast<int>(1280 * hidpi_scale),
                                    static_cast<int>(720 * hidpi_scale), "Axes", nullptr, nullptr);
  AX_THROW_IF_NULLPTR(impl_->window_, "Failed to create GLFW window");

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

  AX_TRACE("Window information: size={}, pos={}, fb_size={}, fb_scale={}", impl_->size_.transpose(),
           impl_->pos_.transpose(), impl_->fb_size_.transpose(), impl_->fb_scale_.transpose());

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
  AX_TRACE("Window created");
}

Window::~Window() {
  if (impl_ != nullptr && impl_->window_ != nullptr) {
    glfwDestroyWindow(impl_->window_);
  }
  glfwTerminate();
  AX_TRACE("Window destroyed");
}

/****************************** Meta Data Getters ******************************/
math::vec2i Window::GetSize() const { return impl_->size_; }

math::vec2i Window::GetPos() const { return impl_->pos_; }

math::vec2i Window::GetFrameBufferSize() const { return impl_->fb_size_; }

math::vec2r Window::GetFrameBufferScale() const { return impl_->fb_scale_.cast<real>(); }

math::vec2r Window::GetCursorPos() const {
  double pos_x, pos_y;
  glfwGetCursorPos(impl_->window_, &pos_x, &pos_y);
  return {pos_x, pos_y};
}

void* Window::GetWindowInternal() const { return impl_->window_; }

void Window::PollEvents() const {
  glfwPollEvents();
  trigger_queue<WindowSizeEvent>();
  trigger_queue<WindowPosEvent>();
  trigger_queue<FrameBufferSizeEvent>();
  trigger_queue<DropEvent>();
  trigger_queue<KeyboardEvent>();
  trigger_queue<CursorMoveEvent>();
  trigger_queue<ScrollEvent>();
  trigger_queue<MouseButtonEvent>();
}

bool Window::ShouldClose() const { return glfwWindowShouldClose(impl_->window_); }

void Window::SwapBuffers() const { glfwSwapBuffers(impl_->window_); }

}  // namespace ax::gl
