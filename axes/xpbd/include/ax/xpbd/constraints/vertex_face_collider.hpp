#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
class Constraint_VertexFaceCollider : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::VertexFaceCollider; }
  Constraint_VertexFaceCollider() = default;
  ~Constraint_VertexFaceCollider() override = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  Real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(Real scale) override;
  void UpdatePositionConsensus() override;

  Real tol_ = 1e-2; ///< Tolerance for the collision detection. cannnot be too small for stability.
  std::vector<math::RealMatrix<3, 4>> dual_;
  std::vector<math::RealMatrix<3, 4>> gap_;
  std::vector<math::RealMatrix<3, 4>> origin_;
  std::vector<Real> stiffness_;
  std::map<std::pair<Index, Index>, Index> collidings_;
  std::map<std::pair<Index, Index>, Index> colliding_map_;
  std::map<Index, Index> global_to_local_;
  std::set<Index> colliding_vertices_;
  Real initial_rho_ = 1e2;
  Index iteration_ = 0;
  Real ratio_ = 1.0;
};

}  // namespace ax::xpbd
