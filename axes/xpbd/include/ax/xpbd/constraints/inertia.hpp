#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {


class Constraint_Inertia final : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kInertia; }

  ConstraintSolution SolveDistributed() override;

  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
 
  math::field3r inertia_position_;
  math::field1r vertex_mass_;

  // x_i: For Inertia constraint, f_i(x_i) = ||x - x_inertia||
  math::field3r dual_;
  // y_i
  math::field3r gap_;
};

}
