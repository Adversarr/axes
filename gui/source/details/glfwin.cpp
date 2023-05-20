#include "agui/details/glfwin.hpp"

static void glfw_error_callback(int errc, const char* desc) {
  fprintf(stderr, "GLFW Error %d: %s\n", errc, desc);
}

namespace axes::gui {

GlfwWindow::GlfwWindow(WindowCreateInfo info) {
  title_ = info.title_;
  width_ = info.width_;
  height_ = info.height_;
  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize glfw library.");
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
  glfwSetErrorCallback(glfw_error_callback);
  window_ = glfwCreateWindow(width_, height_, title_.c_str(), nullptr, nullptr);
  if (window_ == nullptr) {
    throw std::runtime_error("Failed to create glfw window.");
  }
  // Setup Resize callback.
  glfwSetWindowUserPointer(window_, this);
  glfwSetFramebufferSizeCallback(window_, WindowResizeCallback);
  glfwSetCursorPosCallback(window_, CursurCallback);

  if (!glfwVulkanSupported()) {
    throw std::runtime_error("Glfw does not have vulkan support.");
  }
}

GlfwWindow::~GlfwWindow() {
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
    glfwTerminate();
  }
}

GLFWwindow* GlfwWindow::GetWindow() {
  return window_;
}

void GlfwWindow::WindowResizeCallback(GLFWwindow* window, int width,
                                      int height) {
  auto* w = reinterpret_cast<GlfwWindow*>(glfwGetWindowUserPointer(window));
  w->resized_ = true;
  w->height_ = height;
  w->width_ = width;
}

void GlfwWindow::CursurCallback(GLFWwindow* window, double xpos, double ypos) {
  auto* w = reinterpret_cast<GlfwWindow*>(glfwGetWindowUserPointer(window));
  w->mouse_x_position_ = xpos;
  w->mouse_y_position_ = ypos;
}

bool GlfwWindow::ShouldClose() { return glfwWindowShouldClose(window_); }

RealVector2 GlfwWindow::GetCursurPosition() const noexcept {
  return RealVector2{mouse_x_position_, mouse_y_position_};
}

void GlfwWindow::UpdateWindowSize() {
  width_ = height_ = 0;
  while (width_ == 0 || height_ == 0) {
    glfwGetFramebufferSize(window_, &width_, &height_);
    glfwWaitEvents();
  }
}

vk::SurfaceKHR GlfwWindow::CreateSurface(vk::Instance instance) {
  if (!instance) {
    throw std::runtime_error(
        "Failed to create glfw surface: cannot lock `vk::Instance`.");
  }
  VkSurfaceKHR surf;
  glfwCreateWindowSurface(static_cast<VkInstance>(instance), window_, nullptr,
                          &surf);
  return surf;
}

std::pair<UInt32, UInt32> GlfwWindow::GetWindowSize() const noexcept {
  return {width_, height_};
}
}  // namespace axes::gui
