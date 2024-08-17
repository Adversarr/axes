#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_BallCollider final : public ConstraintBase{
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kBallCollider; }
  ~Constraint_BallCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  math::RealVector3 center_ = -math::RealVector3::UnitY();
  real radius_ = 0.9;
  real tol_ = 1e-3;

  std::vector<math::RealVector3> dual_;
  std::vector<math::RealVector3> gap_;
  std::vector<real> stiffness_;
  std::set<Index> collidings_;
  real initial_rho_ = 1e2;
  Index iteration_ = 0;
};
}