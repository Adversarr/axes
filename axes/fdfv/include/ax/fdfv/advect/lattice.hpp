#pragma once
#include "ax/math/lattice.hpp"
#include "backtrace.hpp"

namespace ax::pde {

/**
 * @brief This algorithm solves the advection problem formulated as:
 *          u_t + v . grad(u) = 0
 * The internal implementation is MacCormack's method.
 *   u1(i, j) = u0(i, j) - dt/dx * (v(i, j) * (u0(i+1, j) - u0(i, j)) - u0(i, j) * (v(i, j) - v(i-1,
 * j)))
 *
 * @tparam dim The dimension of the problem.
 */
template <int dim> class AdvectionProblem {
public:
  /**
   * @brief Constructs an AdvectionProblem object.
   *
   * @param dtdx The ratio of time step to spatial step.
   * @param c The advection velocity.
   */
  AdvectionProblem(Real dtdx, Real c) : dtdx_(dtdx), c_(c) {}

  // Instead of v, we solve u_t + u . grad(u) = 0
  // StatusOr<math::StaggeredLattice<dim, Real>> AdvectVelocity(bool periodic);

private:
  // math::StaggeredLattice<dim, Real> velocity_;
  Real dtdx_ = 1.0;
  Real c_ = 1.0;
};

/**
 * @brief Semi-Lagrangian advection method for a scalar lattice.
 *
 * @tparam dim The dimension of the lattice.
 * @tparam T The type of the lattice values.
 * @param u The scalar lattice to advect.
 * @param v The velocity lattice, should not be staggered.
 * @param dt The time step.
 * @return The advected lattice.
 */
template <int dim>
math::Lattice<dim, math::RealVector<dim>> semi_lagrangian(const math::Lattice<dim, math::RealVector<dim>>& u,
                                                    const math::Lattice<dim, math::RealVector<dim>>& v,
                                                    Real dt, bool periodic) {
  math::Lattice<dim, math::RealVector<dim>> output(v.Shape());
  for (auto ijk : math::ndrange<dim>(v.Shape())) {
    math::IndexVector<dim> i = math::tuple_to_vector<Index, dim>(ijk);
    math::RealVector<dim> velocity = v(i);
    math::RealVector<dim> X = backtrace<dim>(i.template cast<Real>(), velocity, dt);
    output(i) = math::lerp_outside<dim>(u, X, periodic);
  }
  return output;
}

template <int dim>
math::Lattice<dim, math::RealVector<dim>> bfecc(const math::Lattice<dim, math::RealVector<dim>>& u,
                                          const math::Lattice<dim, math::RealVector<dim>>& v, Real dt,
                                          bool periodic) {
  math::Lattice<dim, math::RealVector<dim>> x0 = semi_lagrangian<dim>(u, v, dt, periodic);
  math::Lattice<dim, math::RealVector<dim>> x1 = semi_lagrangian<dim>(x0, v, -dt, periodic);
  math::Lattice<dim, math::RealVector<dim>> x2(u.Shape());
  for (auto ijk : math::ndrange<dim>(u.Shape())) {
    math::IndexVector<dim> i = math::tuple_to_vector<Index, dim>(ijk);
    x2(i) = x0(i) + 0.5 * (x1(i) - u(i));
  }
  return x2;
}

/**
 * @brief Semi-Lagrangian advection method for a staggered lattice.
 *
 * @tparam dim The dimension of the lattice.
 * @tparam T The type of the lattice values.
 * @param v The velocity lattice.
 * @param dt The time step.
 * @param periodic Whether the lattice has periodic boundary conditions.
 * @return The advected lattice.
 */
template <int dim, typename T>
math::Lattice<dim, T> semi_lagrangian_staggered(const math::Lattice<dim, math::RealVector<dim>>& v,
                                                Real dt, bool periodic) {
  math::Lattice<dim, T> output(v.Shape());
  T dv = math::make_zeros<T>();
  for (auto ijk : math::ndrange<dim>(v.Shape())) {
    auto i = math::tuple_to_vector(ijk);
    math::RealVector<dim> velocity;
    if_likely(all(i < v.Shape())) { velocity = 0.5 * (v(i) + v(i + math::ones<dim, Index>())); }
    else if (periodic) {
      velocity = v(i) + v(math::imod(i + math::ones<dim, Index>(), v.Shape()));
    }
    else {
      velocity = v(i);
    }
    math::RealVector<dim> X = backtrace(i.template cast<Real>(), velocity, dt);
    for (Index d = 0; d < dim; ++d) {
      math::RealVector<dim> stagger_position = X + math::unit<dim, Real>(d) * 0.5;
      math::IndexVector<dim> sub = to_subscripts(stagger_position);
      math::IndexVector<dim> sub2 = sub + math::unit<dim, Index>(d);
      Real val = 0;
      Real weight = stagger_position[d] - math::cast<Real>(sub[d]);
      if_likely(all(sub < v.Shape()) || periodic) {
        val += v(imod(sub + v.Shape(), v.Shape()))[d] * (1 - weight);
      }
      if_likely(all(sub2 < v.Shape()) || periodic) {
        val += v(imod(sub2 + v.Shape(), v.Shape()))[d] * weight;
      }
      output(i)[d] = val;
    }
  }
  return output;
}

}  // namespace ax::pde