#pragma once
#include "ax/fem/scheme.hpp"

namespace ax::fem {

template <Index dim> class TimestepScheme_BackwardEuler : public TimestepSchemeBase<dim> {
public:
  TimestepScheme_BackwardEuler() = default;
  ~TimestepScheme_BackwardEuler() = default;

  virtual TimestepSchemeKind GetKind() const final { return TimestepSchemeKind::kBackwardEuler; }

  math::spmatr ComposeHessian(math::spmatr const& M, math::spmatr const& K) const final;

  real ComposeEnergy(real inertia_term, real stiffness_term) const final;

  math::RealField<dim> ComposeGradient(math::spmatr const& M,
                                    math::RealField<dim> const& u_next,
                                    math::RealField<dim> const& internal_neg_force,
                                    math::RealField<dim> const& precomputed) const final;

  math::RealField<dim> Precomputed(math::spmatr const& M, math::RealField<dim> const& u_current,
                                math::RealField<dim> const& u_old, math::RealField<dim> const& v_current,
                                math::RealField<dim> const& v_old,
                                math::RealField<dim> const& ext_accel) const final;

  math::RealField<dim> NewVelocity(math::RealField<dim> const& u_current, math::RealField<dim> const& u_old,
                                math::RealField<dim> const& v_current, math::RealField<dim> const& v_old,
                                math::RealField<dim> const& du) const final;
};

}
