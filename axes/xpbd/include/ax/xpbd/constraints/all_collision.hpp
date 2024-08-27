#pragma once
#include <unordered_set>

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_AllCollision : public ConstraintBase {
public:
  // struct Info {
  //   math::RealMatrix<3, 4> dual;
  //   math::RealMatrix<3, 4> gap;
  //   math::RealMatrix<3, 4> origin;
  //   real stiffness;
  //   real rho;
  //   Index prim_a, prim_b;
  //   geo::CollisionKind report_kind;
  //   geo::CollisionKind actual_kind;
  // };

  Constraint_AllCollision() = default;
  ConstraintKind GetKind() const override { return ConstraintKind::Collision; }
  ~Constraint_AllCollision() = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  Real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(Real scale) override;
  void UpdatePositionConsensus() override;

  Real tol_ = 1e-2;  ///< Tolerance for the collision detection. cannnot be too small for stability.
  using I2 = std::pair<Index, Index>;

  std::vector<math::RealMatrix<3, 4>> dual_;
  std::vector<math::RealMatrix<3, 4>> gap_;
  std::vector<math::RealMatrix<3, 4>> origin_;
  std::vector<Real> stiffness_;
  std::set<I2> vt_, ee_;
  std::map<Index, Index> global_to_local_;
  std::set<Index> colliding_vertices_;

  std::vector<geo::CollisionKind> kind_;
  Real initial_rho_ = 1e2;
  Index iteration_ = 0;
  Real ratio_ = 1.1;
};

}  // namespace ax::xpbd
