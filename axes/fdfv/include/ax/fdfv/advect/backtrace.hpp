#pragma once
#include "ax/math/common.hpp"

namespace ax::pde {

/**
 * Backtraces the given position in the specified velocity field for a given time step.
 * 
 * @param X The position to backtrace.
 * @param v The velocity field.
 * @param dt The time step.
 * @return The backtraced position.
 */
template<int dim>
AX_FORCE_INLINE math::RealVector<dim> backtrace(const math::RealVector<dim>& X, const math::RealVector<dim>& v, Real dt) {
  return X - dt * v;
}

template<int dim>
AX_FORCE_INLINE math::IndexVector<dim> to_subscripts(math::RealVector<dim> const& X) {
  return math::floor(X).template cast<Index>();
}

}