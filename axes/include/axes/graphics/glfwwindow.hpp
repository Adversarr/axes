#pragma once
#include <GLFW/glfw3.h>
#include <absl/container/inlined_vector.h>

#include <utility>

#include "axes/core/common.hpp"
#include "axes/math/common.hpp"

namespace axes {
struct KeyboardEvent {
  KeyboardEvent(int key) : key_(key) {}
  int key_;
};

/**
 * @brief A class representing a GLFW window.
 */
class GlfwWindow {
public:
  /**
   * @brief Alias for a 2D vector of unsigned integers representing the window
   * size.
   */
  using WindowSize = std::pair<uint32_t, uint32_t>;

  /**
   * @brief Construct a new Glfw Window object
   *
   */
  GlfwWindow();

  /**
   * @brief Destructor for the GlfwWindow class.
   */
  ~GlfwWindow();

  /**
   * @brief Gets the size of the window.
   *
   * @return The size of the window as a WindowSize object.
   */
  WindowSize GetSize();

  /**
   * @brief Poll events from the window.
   *
   */
  void PollEvents();

  /**
   * @brief Updates the size of the window.
   */
  void UpdateWindowSize();

  /**
   * @brief Checks if a key is currently pressed.
   *
   * @param glfw_key The GLFW key code to check.
   * @return True if the key is currently pressed, false otherwise.
   */
  bool IsKeyPressed(int glfw_key) const noexcept;

  /**
   * @brief Checks if the window has been resized.
   *
   * @return True if the window has been resized, false otherwise.
   */
  bool IsResized() const noexcept;

  /**
   * @brief Resets the resize flag for the window.
   */
  void ResetResizeFlag();

  /**
   * @brief Gets the internal GLFW window object.
   *
   * @return A pointer to the internal GLFW window object.
   */
  GLFWwindow *GetInternal();

  /**
   * @brief Checks if the window should be closed.
   *
   * @return True if the window should be closed, false otherwise.
   */
  bool ShouldClose();

  /**
   * @brief Gets the size of the window.
   *
   * @return A pair of unsigned integers representing the width and height of
   * the window.
   */
  std::pair<UInt32, UInt32> GetWindowSize() noexcept;

  /**
   * @brief Callback function for when the window is resized.
   *
   * @param window The GLFW window object.
   * @param width The new width of the window.
   * @param height The new height of the window.
   */
  static void WindowResizeCallback(GLFWwindow *window, int width, int height);

  std::pair<Int32, Int32> GetCursorPosition() const;

  Int32 GetCursorScoll() const;

  /**
   * @brief Gets the required extensions for GLFW.
   *
   * @return A vector of strings representing the required extensions.
   */
  static std::vector<std::string> GetGlfwRequiredExtensions();

  static void ScrollCallback(GLFWwindow *window, double xoffset, double yoffset);

  static void KeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);

private:
  Vec2r scroll_;
  std::string title_{"axes viewer"}; /**< The title of the window. */
  GLFWwindow *window_{nullptr};      /**< The internal GLFW window object. */
  Real mouse_x_position_;            /**< The x position of the cursor. */
  Real mouse_y_position_;            /**< The y position of the cursor. */
  int width_{1366};                  /**< The width of the window. */
  int height_{768};                  /**< The height of the window. */
  bool resized_{false};              /**< Flag indicating if the window has been resized. */
  absl::InlinedVector<int, 5> key_pressed_;
};

}  // namespace axes
