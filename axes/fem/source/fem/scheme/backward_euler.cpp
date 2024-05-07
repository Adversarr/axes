#include "ax/fem/scheme/backward_euler.hpp"

namespace ax::fem {

template <idx dim>
math::sp_matxxr TimestepScheme_BackwardEuler<dim>::ComposeHessian(math::sp_matxxr const& M,
                                                                  math::sp_matxxr const& K) const {
  real const dt = this->dt_;
  return M + dt * dt * K;
}

template <idx dim> math::fieldr<dim> TimestepScheme_BackwardEuler<dim>::ComposeGradient(
    math::sp_matxxr const& M, math::fieldr<dim> const& u_next,
    math::fieldr<dim> const& internal_neg_force, math::fieldr<dim> const& precomputed) const {
  real const dt = this->dt_;
  return (u_next - precomputed) * M + dt * dt * internal_neg_force;
}

template <idx dim> math::fieldr<dim> TimestepScheme_BackwardEuler<dim>::Precomputed(
    math::sp_matxxr const& M, math::fieldr<dim> const& u_current, math::fieldr<dim> const& u_old,
    math::fieldr<dim> const& v_current, math::fieldr<dim> const& v_old,
    math::fieldr<dim> const& ext_accel) const {
  real const dt = this->dt_;
  return dt * v_current + dt * dt * ext_accel;
}

template <idx dim> math::fieldr<dim> TimestepScheme_BackwardEuler<dim>::NewVelocity(
    math::fieldr<dim> const& u_current, math::fieldr<dim> const& u_old,
    math::fieldr<dim> const& v_current, math::fieldr<dim> const& v_old,
    math::fieldr<dim> const& du) const {
  return du / this->dt_;
}

template <idx dim>
real TimestepScheme_BackwardEuler<dim>::ComposeEnergy(real inertia, real stiffness) const {
  return inertia + stiffness * this->dt_ * this->dt_;
}

template class TimestepScheme_BackwardEuler<2>;
template class TimestepScheme_BackwardEuler<3>;

}  // namespace ax::fem
