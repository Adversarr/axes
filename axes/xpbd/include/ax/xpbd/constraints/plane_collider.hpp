#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {


class Constraint_PlaneCollider final : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kPlaneCollider; }
  ~Constraint_PlaneCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  Real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(Real scale) override;
  void UpdatePositionConsensus() override;

  math::RealVector3 normal_ = math::RealVector3::UnitY();
  Real offset_ = -1;
  Real tol_ = 1e-3;

  std::vector<math::RealVector3> dual_;
  std::vector<math::RealVector3> gap_;
  std::vector<Real> stiffness_;
  std::set<Index> collidings_;
  Real initial_rho_ = 1e5;
  Index iteration_ = 0;
};
}