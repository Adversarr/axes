#pragma once

#include "ax/math/common.hpp"
#include "ax/utils/common.hpp"
namespace ax::gl {

/****************************** Events ******************************/

struct WindowSizeEvent {
  math::IndexVec2 size_;
};

struct WindowPosEvent {
  math::IndexVec2 pos_;
};

struct FrameBufferSizeEvent {
  math::IndexVec2 size_;
};

struct DropEvent {
  std::vector<std::string> paths_;
};

struct KeyboardEvent {
  int key_;
  int scancode_;
  int action_;
  int mods_;
};

struct CursorMoveEvent {
  math::RealVector2 pos_;
};

struct ScrollEvent {
  math::RealVector2 offset_;
};

struct MouseButtonEvent {
  int button_;
  int action_;
  int mods_;
};

/****************************** Window ******************************/
class Window {
public:
  struct Impl;

  // NOTE: The constructor will automatically check if there is more than one Window
  //       instance. We do not allow more than one Window instance.
  Window();
  ~Window();
  AX_DECLARE_CONSTRUCTOR(Window, delete, default);

  /****************************** Meta Data Getters ******************************/

  math::IndexVec2 GetSize() const;

  math::IndexVec2 GetPos() const;

  math::IndexVec2 GetFrameBufferSize() const;

  math::RealVector2 GetFrameBufferScale() const;

  math::RealVector2 GetCursorPos() const;

  void* GetWindowInternal() const;

  void PollEvents() const;

  void SwapBuffers() const;

  bool ShouldClose() const;

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace ax::gl
