#pragma once
#include "ax/utils/common.hpp"
namespace ax::gl {

template <typename Bindable> struct WithBind {
  explicit WithBind(Bindable& bindable) : bindable_{bindable} { bindable_.Bind(); }
  ~WithBind() { bindable_.Unbind(); }
  template <typename Fn> decltype(auto) operator()(Fn&& fn) const { return fn(); }
  Bindable& bindable_;
};

#define AXGL_WITH_BIND(bindable) if (auto _ = ax::gl::WithBind{bindable}; true)

}  // namespace ax::gl
