#pragma once
#include "axes/core/status.hpp"
#include "axes/utils/common.hpp"
namespace ax::gl {

template <typename Bindable> class BindGuard {
public:
  BindGuard(Bindable& bindable) : bindable(&bindable) {}
  AX_DECLARE_COPY_CTOR(BindGuard, delete);
  BindGuard(BindGuard&& other) noexcept : bindable(other.bindable) { other.bindable = nullptr; }

  Status Bind() { return bindable->Bind(); }

  ~BindGuard() {
    if (bindable) CHECK_OK(bindable->Unbind());
  }

private:
  Bindable* bindable;
};

template <typename Bindable> std::pair<Status, BindGuard<Bindable>> bind(Bindable& bindable) {
  Status status = bindable.Bind();
  return {status, BindGuard<Bindable>(bindable)};
}

#define AXGL_WITH_BINDC(bindable)                                         \
  if (auto [status, bind_guard] = ax::gl::bind(bindable); !status.ok()) { \
    CHECK_OK(status);                                                     \
  } else

#define AXGL_WITH_BINDR(bindable)                                         \
  if (auto [status, bind_guard] = ax::gl::bind(bindable); !status.ok()) { \
    return status;                                                        \
  } else

}  // namespace ax::gl
