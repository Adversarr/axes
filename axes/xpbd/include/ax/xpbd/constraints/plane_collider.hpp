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

  math::vec3r normal_ = math::vec3r::UnitY();
  real offset_ = -1;
  real tol_ = 1e-3;

  std::vector<math::vec3r> dual_;
  std::vector<math::vec3r> gap_;
  std::vector<real> stiffness_;
  std::set<idx> collidings_;
  real initial_rho_ = 1e5;
  idx iteration_ = 0;
};
}