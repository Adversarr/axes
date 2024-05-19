#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
class Constraint_Spring final : public ConstraintBase {
public:
  Constraint_Spring() = default;

  void SetSprings(math::field2i const& indices, math::field1r const& stiffness);

  ConstraintKind GetKind() const override { return ConstraintKind::kSpring; }

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;


  math::field1r spring_stiffness_;
  math::field1r spring_length_;

  // x_i
  math::fieldr<6> dual_;
  math::fieldr<6> gap_;
};
}  // namespace ax::xpbd
