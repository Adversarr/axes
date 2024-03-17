#pragma once
#include "axes/math/lattice.hpp"

namespace ax::pde {

/**
 * @brief This algorithm solves the advection problem formulated as:
 *          u_t + v . grad(u) = 0
 * The internal implementation is MacCormack's method.
 *   u1(i, j) = u0(i, j) - dt/dx * (v(i, j) * (u0(i+1, j) - u0(i, j)) - u0(i, j) * (v(i, j) - v(i-1, j)))
 *
 * @tparam dim
 */
template <idx dim> class AdvectionProblem {
public:
  AdvectionProblem(real dtdx, real c): dtdx_(dtdx), c_(c) {}

  // Instead of v, we solve u_t + u . grad(u) = 0
  // StatusOr<math::StaggeredLattice<dim, real>> AdvectVelocity(bool periodic);

private:
  // math::StaggeredLattice<dim, real> velocity_;
  real dtdx_=1.0;
  real c_ = 1.0;
};

}  // namespace ax::pde