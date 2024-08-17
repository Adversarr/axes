#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {


class Constraint_PlaneCollider final : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kPlaneCollider; }
  ~Constraint_PlaneCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  math::RealVector3 normal_ = math::RealVector3::UnitY();
  real offset_ = -1;
  real tol_ = 1e-3;

  std::vector<math::RealVector3> dual_;
  std::vector<math::RealVector3> gap_;
  std::vector<real> stiffness_;
  std::set<Index> collidings_;
  real initial_rho_ = 1e5;
  Index iteration_ = 0;
};
}