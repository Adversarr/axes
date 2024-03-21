/**
 * NOTE: We only care about Real elements, so we don't need to template for the
 * scalar type.
 *
 * NOTE: We only implement three types of elements:
 * 1. Pk.
 * 2. Qk. K=1
 * 3. Hermite
 *
 */

#pragma once
#include "axes/math/common.hpp"

namespace ax::pde::elements {

template <typename Derived> struct ConformingElementBase {
  using impl_type = Derived;

  // scalar type of the element. Could be real or complex.
  using scalar_type = typename Derived::scalar_type;
};

}  // namespace ax::pde::elements
