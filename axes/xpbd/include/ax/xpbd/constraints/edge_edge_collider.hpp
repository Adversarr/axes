#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_EdgeEdgeCollider : public ConstraintBase {
public:
  Constraint_EdgeEdgeCollider() = default;
  ConstraintKind GetKind() const override { return ConstraintKind::kEdgeEdgeCollider; }
  ~Constraint_EdgeEdgeCollider() = default;

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
  std::set<std::pair<Index, Index>> collidings_;
  std::map<Index, Index> global_to_local_;
  std::set<Index> colliding_vertices_;
  Real initial_rho_ = 1e2;
  Index iteration_ = 0;
  Real ratio_ = 1.1;
};

}  // namespace ax::xpbd
