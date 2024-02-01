#pragma once

#include "axes/math/common.hpp"
namespace ax::math {

template <typename Derived> class LinearOperatorBase {
public:
  vecxr operator*(vecxr const& x) const { return static_cast<Derived const&>(*this).Eval(x); }

  vecxr operator()(vecxr const& x) const { return operator*(x); }
};

}  // namespace ax::math
