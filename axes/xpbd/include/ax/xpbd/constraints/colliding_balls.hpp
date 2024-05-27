#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd{

class Constraint_CollidingBalls : public ConstraintBase {
public:
  Constraint_CollidingBalls() = default;
  ~Constraint_CollidingBalls() = default;

  ConstraintKind GetKind() const override { return ConstraintKind::kCollidingBalls; }

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  real ball_radius_ = 3e-2; ///< Tolerance for the collision detection. cannnot be too small for stability.
  std::vector<math::matr<3, 2>> dual_;
  std::vector<math::matr<3, 2>> gap_;
  std::vector<real> stiffness_;

  std::map<std::pair<idx, idx>, idx> collidings_;
  std::map<std::pair<idx, idx>, idx> colliding_map_;
  std::map<idx, idx> global_to_local_;
  std::set<idx> colliding_vertices_;
  idx iteration_;
  real tol_ = 1e-3;
};
}