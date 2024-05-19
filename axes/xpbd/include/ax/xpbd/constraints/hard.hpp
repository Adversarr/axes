#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_Hard final : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kHard; }

  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  ConstraintSolution SolveDistributed() override;

  void SetHard(math::field1i const& indices,
               math::field3r const& target_position);
  void SetHard(math::field1i const& indices);

  // x_i: For hard constraint, just the target position because
  //      f_i is the indicator fn.
  math::field3r dual_;
  math::field3r gap_;
  real initial_rho_ = 1e3;
};

}  // namespace ax::xpbd
