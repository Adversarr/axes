#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
class Constraint_VertexFaceCollider : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kBallCollider; }
  Constraint_VertexFaceCollider() = default;
  ~Constraint_VertexFaceCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  real tol_ = 3e-3;
  List<math::matr<3, 4>> dual_;
  List<math::matr<3, 4>> gap_;
  List<math::matr<3, 4>> origin_;
  List<real> stiffness_;
  std::set<std::pair<idx, idx>> collidings_;
  std::map<idx, idx> global_to_local_;
  std::set<idx> colliding_vertices_;
  real initial_rho_ = 1e4;
  idx iteration_ = 0;
  real ratio_ = 1.4;
};

}  // namespace ax::xpbd
