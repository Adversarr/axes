#pragma once
#include "ax/math/sparse.hpp"
#include "ax/utils/enum_refl.hpp"

namespace ax::fem {

AX_DEFINE_ENUM_CLASS(TimestepSchemeKind, kBackwardEuler, kBDF2);

template <int dim> class TimestepSchemeBase {
public:
  TimestepSchemeBase() = default;
  virtual ~TimestepSchemeBase() = default;

  virtual TimestepSchemeKind GetKind() const = 0;

  static std::unique_ptr<TimestepSchemeBase<dim>> Create(TimestepSchemeKind kind);

  void SetDeltaT(Real dt) { dt_ = dt; }

  virtual math::RealSparseMatrix ComposeHessian(math::RealSparseMatrix const& M, math::RealSparseMatrix const& K) const
      = 0;

  virtual math::RealField<dim> ComposeGradient(math::RealSparseMatrix const& M,
                                            math::RealField<dim> const& u_next,
                                            math::RealField<dim> const& internal_force,
                                            math::RealField<dim> const& precomputed) const
      = 0;

  virtual Real ComposeEnergy(Real inertia_term, Real stiffness_term) const = 0;

  virtual math::RealField<dim> Precomputed(math::RealSparseMatrix const& M,
                                        math::RealField<dim> const& u_current,
                                        math::RealField<dim> const& u_old,
                                        math::RealField<dim> const& v_current,
                                        math::RealField<dim> const& v_old,
                                        math::RealField<dim> const& ext_accel) const
      = 0;

  virtual math::RealField<dim> NewVelocity(math::RealField<dim> const& u_current,
                                        math::RealField<dim> const& u_old,
                                        math::RealField<dim> const& v_current,
                                        math::RealField<dim> const& v_old,
                                        math::RealField<dim> const& du) const
      = 0;

protected:
  Real dt_;
};

}  // namespace ax::fem
