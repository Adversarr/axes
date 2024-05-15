#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
template <idx dim>
class Constraint_Spring final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kSpring; }

  ConstraintSolution<dim> SolveConsensus() override;
  void BeginStep() override;
  void UpdateDuality() override;
  void EndStep() override;

  math::field1r spring_stiffness_;

  // x_i
  math::fieldr<dim> dual_;
  // y_i
  math::fieldr<dim> gap_;
};
}