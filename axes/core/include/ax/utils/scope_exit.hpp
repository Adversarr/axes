#pragma once
#include <functional>

namespace ax::utils {

template <typename Fn> struct ScopeExit {
public:
  ScopeExit(Fn&& fn) noexcept : fn_(std::forward<Fn>(fn)) {}
  ~ScopeExit() noexcept {
    if (!dismissed_) fn_();
  }

  ScopeExit(ScopeExit const&) = delete;
  void operator=(ScopeExit const&) = delete;
  ScopeExit(ScopeExit&&) = delete;
  void operator=(ScopeExit&&) = delete;

  void Dismiss() { dismissed_ = true; }

private:
  std::function<void()> fn_;
  bool dismissed_ = false;
};

template <typename Fn> auto make_scope_exit(Fn&& fn) { return ScopeExit<Fn>(std::forward<Fn>(fn)); }

}  // namespace ax::utils