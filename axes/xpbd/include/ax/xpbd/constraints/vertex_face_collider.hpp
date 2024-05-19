#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
template<idx dim>
class Constraint_VertexFaceCollider : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kBallCollider; }
  Constraint_VertexFaceCollider() = default;
  ~Constraint_VertexFaceCollider() override = default;

  ConstraintSolution<dim> SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  math::vecr<dim> center_ = -math::vecr<dim>::UnitY();
  real radius_ = 0.9;
  real tol_ = 1e-3;

  List<math::vecr<dim>> dual_;
  List<math::vecr<dim>> gap_;
  List<real> stiffness_;
  std::set<idx> collidings_;
  real initial_rho_ = 1e6;
  idx iteration_ = 0;
};

}