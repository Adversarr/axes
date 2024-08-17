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
 
  math::RealField3 inertia_position_;
  math::RealField1 vertex_mass_;

  // x_i: For Inertia constraint, f_i(x_i) = ||x - x_inertia||
  math::RealField3 dual_;
  // y_i
  math::RealField3 gap_;
};

}
