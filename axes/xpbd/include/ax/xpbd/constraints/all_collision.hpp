#pragma once
#include <unordered_set>

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_AllCollision : public ConstraintBase {
public:
  // struct Info {
  //   math::matr<3, 4> dual;
  //   math::matr<3, 4> gap;
  //   math::matr<3, 4> origin;
  //   real stiffness;
  //   real rho;
  //   idx prim_a, prim_b;
  //   geo::CollisionKind report_kind;
  //   geo::CollisionKind actual_kind;
  // };

  Constraint_AllCollision() = default;
  ConstraintKind GetKind() const override { return ConstraintKind::kCollision; }
  ~Constraint_AllCollision() = default;

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void UpdatePositionConsensus() override;

  real tol_ = 1e-2;  ///< Tolerance for the collision detection. cannnot be too small for stability.
  using I2 = std::pair<idx, idx>;

  List<math::matr<3, 4>> dual_;
  List<math::matr<3, 4>> gap_;
  List<math::matr<3, 4>> origin_;
  List<real> stiffness_;
  std::set<I2> vt_, ee_;
  std::map<idx, idx> global_to_local_;
  std::set<idx> colliding_vertices_;

  std::vector<geo::CollisionKind> kind_;
  real initial_rho_ = 1e2;
  idx iteration_ = 0;
  real ratio_ = 1.1;
};

}  // namespace ax::xpbd
