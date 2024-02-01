#pragma once

#include "axes/math/common.hpp"
#include "axes/utils/common.hpp"
namespace ax::gui {

/****************************** Events ******************************/

struct WindowSizeEvent {
  math::vec2i size_;
};

struct WindowSizePos {
  math::vec2i pos_;
};

struct FrameBufferSizeEvent {
  math::vec2i size_;
};

struct FrameBufferScaleEvent {
  math::vec2r scale_;
};

struct DropEvent {
  std::vector<std::string> paths_;
};

struct KeyboardEvent {
  int key;
  int scancode;
  int action;
  int mods;
};

struct CursorMove {
  math::vec2r pos_;
  math::vec2r delta_;
};

struct ScrollEvent {
  math::vec2r offset_;
};

struct MouseButtonEvent {
  int button;
  int action;
  int mods;
};

/****************************** Window ******************************/
class Window {
public:
  struct Impl;

  // NOTE: The constructor will automatically check if there is more than one Window
  //       instance. We do not allow more than one Window instance.
  Window();

  /****************************** Meta Data Getters ******************************/

  math::vec2i GetSize() const;

  math::vec2i GetPos() const;

  math::vec2i GetFrameBufferSize() const;

  math::vec2r GetFrameBufferScale() const;

  math::vec2r GetCursorPos() const;

private:
  utils::uptr<Impl> impl_;
};

}  // namespace ax::gui
