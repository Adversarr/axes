#pragma once
#include "ax/fem/scheme.hpp"

namespace ax::fem {

template <idx dim> class TimestepScheme_BackwardEuler : public TimestepSchemeBase<dim> {
public:
  TimestepScheme_BackwardEuler() = default;
  ~TimestepScheme_BackwardEuler() = default;

  virtual TimestepSchemeKind GetKind() const final { return TimestepSchemeKind::kBackwardEuler; }

  math::sp_matxxr ComposeHessian(math::sp_matxxr const& M, math::sp_matxxr const& K) const final;

  real ComposeEnergy(real inertia_term, real stiffness_term) const final;

  math::fieldr<dim> ComposeResidual(math::sp_matxxr const& M,
                                    math::fieldr<dim> const& x_next,
                                    math::fieldr<dim> const& internal_neg_force,
                                    math::fieldr<dim> const& precomputed) const final;

  math::fieldr<dim> Precomputed(math::sp_matxxr const& M,
                                math::fieldr<dim> const& x_current,
                                math::fieldr<dim> const& x_old,
                                math::fieldr<dim> const& v_current,
                                math::fieldr<dim> const& v_old,
                                math::fieldr<dim> const& ext_accel) const final;

  math::fieldr<dim> InitialGuess(math::fieldr<dim> const& x_current, math::fieldr<dim> const& x_old,
                                 math::fieldr<dim> const& v_current, math::fieldr<dim> const& v_old,
                                 math::fieldr<dim> const& ext_accel) const final;

  math::fieldr<dim> NewVelocity(math::fieldr<dim> const& x_current, math::fieldr<dim> const& x_old,
                                math::fieldr<dim> const& v_current, math::fieldr<dim> const& v_old,
                                math::fieldr<dim> const& du) const final;
};

}