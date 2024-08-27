#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_BallCollider final : public ConstraintBase{
public:
  ConstraintKind GetKind() const override { return ConstraintKind::BallCollider; }
  ~Constraint_BallCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  Real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(Real scale) override;
  void UpdatePositionConsensus() override;

  math::RealVector3 center_ = -math::RealVector3::UnitY();
  Real radius_ = 0.9;
  Real tol_ = 1e-3;

  std::vector<math::RealVector3> dual_;
  std::vector<math::RealVector3> gap_;
  std::vector<Real> stiffness_;
  std::set<Index> collidings_;
  Real initial_rho_ = 1e2;
  Index iteration_ = 0;
};
}