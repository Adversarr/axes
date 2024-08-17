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

  real ball_radius_ = 1e-1; ///< Tolerance for the collision detection. cannnot be too small for stability.
  std::vector<math::RealMatrix<3, 2>> dual_;
  std::vector<math::RealMatrix<3, 2>> gap_;
  std::vector<real> stiffness_;
  std::vector<math::RealMatrix<3, 2>> origin_;

  std::map<std::pair<Index, Index>, Index> collidings_;
  std::map<std::pair<Index, Index>, Index> colliding_map_;
  std::map<Index, Index> global_to_local_;
  std::set<Index> colliding_vertices_;
  Index iteration_;
  real initial_rho_ = 1e4;
  real tol_ = 1e-4;
};
}
