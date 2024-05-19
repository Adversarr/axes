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

  math::vec3r center_ = -math::vec3r::UnitY();
  real radius_ = 0.9;
  real tol_ = 1e-3;

  List<math::vec3r> dual_;
  List<math::vec3r> gap_;
  List<real> stiffness_;
  std::set<idx> collidings_;
  real initial_rho_ = 1e6;
  idx iteration_ = 0;
};
}