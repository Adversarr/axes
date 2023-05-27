#pragma once
#include <string>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <vulkan/vulkan.hpp>

#include "axes/core/common.hpp"
#include "axes/core/math/common.hpp"
#include "common.hpp"  // IWYU pragma: export

#ifndef AXES_DEFAULT_WINDOW_WIDTH
#  define AXES_DEFAULT_WINDOW_WIDTH 1366
#endif

#ifndef AXES_DEFAULT_WINDOW_HEIGHT
#  define AXES_DEFAULT_WINDOW_HEIGHT 768
#endif

namespace axes::gui {

struct WindowCreateInfo {
  std::string title_{"Axes"};
  int width_ = AXES_DEFAULT_WINDOW_WIDTH;
  int height_ = AXES_DEFAULT_WINDOW_HEIGHT;
};

class GlfwWindow {
public:
  explicit GlfwWindow(WindowCreateInfo info);

  ~GlfwWindow();

  void UpdateWindowSize();

  bool IsKeyPressed(int glfw_key) const;

  bool IsResized() const noexcept;

  void ResetResizeFlag();
  /**
   * @brief Returns the internal glfw pointer
   *
   * @return
   */
  GLFWwindow* GetWindow();

  /**
   * @brief window close
   *
   * @return
   */
  bool ShouldClose();

  std::pair<UInt32, UInt32> GetWindowSize() const noexcept;

  vk::SurfaceKHR CreateSurface(vk::Instance instance);

  static void WindowResizeCallback(GLFWwindow* window, int width, int height);

  static void CursurCallback(GLFWwindow* window, double xpos, double ypos);

  RealVector2 GetCursurPosition() const noexcept;

private:
  std::string title_{"Axes"};

  GLFWwindow* window_{nullptr};
  Real mouse_x_position_;
  Real mouse_y_position_;
  int width_{AXES_DEFAULT_WINDOW_WIDTH};
  int height_{AXES_DEFAULT_WINDOW_HEIGHT};
  bool resized_{false};
};

}  // namespace axes::gui
