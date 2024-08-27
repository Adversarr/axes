#pragma once
#include "ax/fem/scheme.hpp"

namespace ax::fem {

template <int dim> class TimestepScheme_BackwardEuler : public TimestepSchemeBase<dim> {
public:
  TimestepScheme_BackwardEuler() = default;

  ~TimestepScheme_BackwardEuler() override = default;

  virtual TimestepSchemeKind GetKind() const final { return TimestepSchemeKind::kBackwardEuler; }

  math::RealSparseMatrix ComposeHessian(math::RealSparseMatrix const& M, math::RealSparseMatrix const& K) const final;

  Real ComposeEnergy(Real inertia_term, Real stiffness_term) const final;

  math::RealField<dim> ComposeGradient(math::RealSparseMatrix const& M,
                                    math::RealField<dim> const& u_next,
                                    math::RealField<dim> const& internal_neg_force,
                                    math::RealField<dim> const& precomputed) const final;

  math::RealField<dim> Precomputed(math::RealSparseMatrix const& M, math::RealField<dim> const& u_current,
                                math::RealField<dim> const& u_old, math::RealField<dim> const& v_current,
                                math::RealField<dim> const& v_old,
                                math::RealField<dim> const& ext_accel) const final;

  math::RealField<dim> NewVelocity(math::RealField<dim> const& u_current, math::RealField<dim> const& u_old,
                                math::RealField<dim> const& v_current, math::RealField<dim> const& v_old,
                                math::RealField<dim> const& du) const final;
};

}
