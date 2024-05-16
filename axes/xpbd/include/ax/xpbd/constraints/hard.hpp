#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

template <idx dim> class Constraint_Hard final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kHard; }

  void BeginStep() override;
  void UpdateDuality() override;
  void EndStep() override;
  ConstraintSolution<dim> SolveDistributed() override;

  // x_i: For hard constraint, just the target position because
  //      f_i is the indicator fn.
  math::fieldr<dim> dual_;
  // y_i
  math::fieldr<dim> gap_;
};

}  // namespace ax::xpbd
