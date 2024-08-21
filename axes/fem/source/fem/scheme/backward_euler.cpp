#include "ax/fem/scheme/backward_euler.hpp"

namespace ax::fem {

template <int dim>
math::RealSparseMatrix TimestepScheme_BackwardEuler<dim>::ComposeHessian(math::RealSparseMatrix const& M,
                                                                  math::RealSparseMatrix const& K) const {
  Real const dt = this->dt_;
  return M + dt * dt * K;
}

template <int dim> math::RealField<dim> TimestepScheme_BackwardEuler<dim>::ComposeGradient(
    math::RealSparseMatrix const& M, math::RealField<dim> const& u_next,
    math::RealField<dim> const& internal_neg_force, math::RealField<dim> const& precomputed) const {
  Real const dt = this->dt_;
  return (u_next - precomputed) * M + dt * dt * internal_neg_force;
}

template <int dim> math::RealField<dim> TimestepScheme_BackwardEuler<dim>::Precomputed(
    math::RealSparseMatrix const& /* M */, math::RealField<dim> const& /* u_current */, math::RealField<dim> const& /* u_old */,
    math::RealField<dim> const& v_current, math::RealField<dim> const& /* v_old */,
    math::RealField<dim> const& ext_accel) const {
  Real const dt = this->dt_;
  return dt * v_current + dt * dt * ext_accel;
}

template <int dim> math::RealField<dim> TimestepScheme_BackwardEuler<dim>::NewVelocity(
    math::RealField<dim> const& /* u_current */, math::RealField<dim> const& /* u_old */,
    math::RealField<dim> const& /* v_current */, math::RealField<dim> const& /* v_old */,
    math::RealField<dim> const& du) const {
  return du / this->dt_;
}

template <int dim>
Real TimestepScheme_BackwardEuler<dim>::ComposeEnergy(Real inertia, Real stiffness) const {
  return inertia + stiffness * this->dt_ * this->dt_;
}

template class TimestepScheme_BackwardEuler<2>;
template class TimestepScheme_BackwardEuler<3>;

}  // namespace ax::fem
