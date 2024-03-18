#pragma once
#include "axes/math/common.hpp"

namespace ax::pde {

/**
 * Backtraces the given position in the specified velocity field for a given time step.
 * 
 * @param X The position to backtrace.
 * @param v The velocity field.
 * @param dt The time step.
 * @return The backtraced position.
 */
template<idx dim>
AX_FORCE_INLINE math::vecr<dim> backtrace(const math::vecr<dim>& X, const math::vecr<dim>& v, real dt) {
  return X - dt * v;
}

template<idx dim>
AX_FORCE_INLINE math::veci<dim> to_subscripts(math::vecr<dim> const& X) {
  return math::floor(X).template cast<idx>();
}

}