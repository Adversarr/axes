#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
class Constraint_VertexFaceCollider : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kVertexFaceCollider; }
  Constraint_VertexFaceCollider() = default;
  ~Constraint_VertexFaceCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  real tol_ = 1e-2; ///< Tolerance for the collision detection. cannnot be too small for stability.
  std::vector<math::matr<3, 4>> dual_;
  std::vector<math::matr<3, 4>> gap_;
  std::vector<math::matr<3, 4>> origin_;
  std::vector<real> stiffness_;
  std::map<std::pair<idx, idx>, idx> collidings_;
  std::map<std::pair<idx, idx>, idx> colliding_map_;
  std::map<idx, idx> global_to_local_;
  std::set<idx> colliding_vertices_;
  real initial_rho_ = 1e2;
  idx iteration_ = 0;
  real ratio_ = 1.0;
};

}  // namespace ax::xpbd
