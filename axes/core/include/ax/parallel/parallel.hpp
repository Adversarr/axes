#pragma once

#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/parallel_invoke.h>

#include "ax/math/shape.hpp"

namespace ax::par {

template <typename Derived>
class LauncherBase {
public:
  LauncherBase() = default;
  virtual ~LauncherBase() = default;

  template <typename Index, int dim, typename Fn, typename... Args>
  AX_HOST void ForEachIndex(const math::Shape<Index, dim>& s, Fn&& fn, Args&&... args) {
    static_cast<Derived*>(this)->ForeachImpl(s, std::forward<Fn>(fn), std::forward<Args>(args)...);
  }

  template <typename Fn, typename... Args>
  AX_HOST void Invoke(Fn&& fn, Args&&... args) {
    static_cast<Derived*>(this)->InvokeImpl(std::forward<Fn>(fn), std::forward<Args>(args)...);
  }
};

}  // namespace ax::par