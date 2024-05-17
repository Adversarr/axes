#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

template <idx dim> using ColliderTestFn = std::function<math::vecr<dim>(math::vecr<dim> const&)>;

template <idx dim>
class Constraint_StaticCollider final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kCollision; }
  ~Constraint_StaticCollider() override = default;

  ConstraintSolution<dim> SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  ColliderTestFn<dim> test_fn_;
  List<math::vecr<dim>> dual_;
  List<math::vecr<dim>> gap_;
  std::set<idx> collidings_;
  real initial_rho_ = 1e3;
};
}