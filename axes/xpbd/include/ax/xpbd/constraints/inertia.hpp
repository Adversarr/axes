#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

template<idx dim>
class Constraint_Inertia final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kInertia; }

  ConstraintSolution<dim> SolveDistributed() override;

  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
 
  math::fieldr<dim> inertia_position_;
  math::field1r vertex_mass_;

  // x_i: For Inertia constraint, f_i(x_i) = ||x - x_inertia||
  math::fieldr<dim> dual_;
  // y_i
  math::fieldr<dim> gap_;
};

}
