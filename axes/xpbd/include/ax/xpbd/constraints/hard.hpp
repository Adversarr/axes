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

  void SetHard(math::field1i const& indices,
               math::fieldr<dim> const& target_position);
  void SetHard(math::field1i const& indices);

  // x_i: For hard constraint, just the target position because
  //      f_i is the indicator fn.
  math::fieldr<dim> dual_;
  // y_i
  math::fieldr<dim> gap_;
  real initial_rho_ = 1e3;
};

}  // namespace ax::xpbd
