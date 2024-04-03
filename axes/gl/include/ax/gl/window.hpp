#pragma once

#include "ax/math/common.hpp"
#include "ax/utils/common.hpp"
namespace ax::gl {

/****************************** Events ******************************/

struct WindowSizeEvent {
  math::vec2i size_;
};

struct WindowPosEvent {
  math::vec2i pos_;
};

struct FrameBufferSizeEvent {
  math::vec2i size_;
};

struct DropEvent {
  List<std::string> paths_;
};

struct KeyboardEvent {
  int key_;
  int scancode_;
  int action_;
  int mods_;
};

struct CursorMoveEvent {
  math::vec2r pos_;
};

struct ScrollEvent {
  math::vec2r offset_;
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

  math::vec2i GetSize() const;

  math::vec2i GetPos() const;

  math::vec2i GetFrameBufferSize() const;

  math::vec2r GetFrameBufferScale() const;

  math::vec2r GetCursorPos() const;

  void* GetWindowInternal() const;

  void PollEvents() const;

  void SwapBuffers() const;

  bool ShouldClose() const;

private:
  UPtr<Impl> impl_;
};

}  // namespace ax::gl
