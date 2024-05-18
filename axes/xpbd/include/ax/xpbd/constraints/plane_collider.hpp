#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

template <idx dim>
class Constraint_PlaneCollider final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kPlaneCollider; }
  ~Constraint_PlaneCollider() override = default;

  ConstraintSolution<dim> SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  math::vecr<dim> normal_ = math::vecr<dim>::UnitY();
  real offset_ = -1;
  real tol_ = 1e-3;

  List<math::vecr<dim>> dual_;
  List<math::vecr<dim>> gap_;
  List<real> stiffness_;
  std::set<idx> collidings_;
  real initial_rho_ = 1e6;
  real k = 0;
  idx iteration_ = 0;
};
}