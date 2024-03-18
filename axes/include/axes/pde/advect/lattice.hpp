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

template<idx dim, typename T>
math::Lattice<dim, T> semi_lagrangian(const math::Lattice<dim, T>& u, const math::Lattice<dim, math::vecr<dim>>& v,
                                      real dt, bool periodic) {
  math::Lattice<dim, T> output(u.Shape());
  T dv; math::zeros_(dv);
  for (auto ijk : math::ndrange<dim>(u.Shape())) {
    auto i = math::tuple_to_vector(ijk);
    // TODO: Implement staggerred version.
    if (u.IsStaggered()) {
      // auto vel = interpolate<dim>(velocity_, X, periodic);
      // output(i) = math::lerp_outside(u, X, periodic, dv, math::staggered);
    } else {
      math::vecr<dim> X = i.template cast<real>() - dt * v(i);
      output(i) = math::lerp_outside(u, X, periodic, dv, math::cell_center);
    }
  }
  return output;
}

}  // namespace ax::pde