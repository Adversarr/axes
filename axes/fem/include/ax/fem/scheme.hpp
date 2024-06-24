#pragma once
#include "ax/math/sparse.hpp"
#include "ax/utils/enum_refl.hpp"

namespace ax::fem {

BOOST_DEFINE_ENUM_CLASS(TimestepSchemeKind, kBackwardEuler, kBDF2);

template <idx dim> class TimestepSchemeBase {
public:
  TimestepSchemeBase() = default;
  virtual ~TimestepSchemeBase() = default;

  virtual TimestepSchemeKind GetKind() const = 0;

  static UPtr<TimestepSchemeBase<dim>> Create(TimestepSchemeKind kind);

  void SetDeltaT(real dt) { dt_ = dt; }

  virtual math::spmatr ComposeHessian(math::spmatr const& M, math::spmatr const& K) const
      = 0;

  virtual math::fieldr<dim> ComposeGradient(math::spmatr const& M,
                                            math::fieldr<dim> const& u_next,
                                            math::fieldr<dim> const& internal_force,
                                            math::fieldr<dim> const& precomputed) const
      = 0;

  virtual real ComposeEnergy(real inertia_term, real stiffness_term) const = 0;

  virtual math::fieldr<dim> Precomputed(math::spmatr const& M,
                                        math::fieldr<dim> const& u_current,
                                        math::fieldr<dim> const& u_old,
                                        math::fieldr<dim> const& v_current,
                                        math::fieldr<dim> const& v_old,
                                        math::fieldr<dim> const& ext_accel) const
      = 0;

  virtual math::fieldr<dim> NewVelocity(math::fieldr<dim> const& u_current,
                                        math::fieldr<dim> const& u_old,
                                        math::fieldr<dim> const& v_current,
                                        math::fieldr<dim> const& v_old,
                                        math::fieldr<dim> const& du) const
      = 0;

protected:
  real dt_;
};

}  // namespace ax::fem
