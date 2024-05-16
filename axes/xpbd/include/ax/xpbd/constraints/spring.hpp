#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
template <idx dim> class Constraint_Spring final : public ConstraintBase<dim> {
public:
  Constraint_Spring() = default;

  void SetSprings(math::field2i const& indices, math::field1r const& stiffness);

  ConstraintKind GetKind() const override { return ConstraintKind::kSpring; }

  ConstraintSolution<dim> SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;


  math::field1r spring_stiffness_;
  math::field1r spring_length_;

  // x_i
  math::fieldr<dim * 2> dual_;
  math::fieldr<dim * 2> dual_old_;

  // y_i
  math::fieldr<dim * 2> gap_;
};
}  // namespace ax::xpbd
