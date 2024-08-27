#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_Hard final : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::Hard; }

  void BeginStep() override;
  Real UpdateDuality() override;
  void EndStep() override;
  ConstraintSolution SolveDistributed() override;

  void SetHard(math::IndexField1 const& indices,
               math::RealField3 const& target_position);
  void SetHard(math::IndexField1 const& indices);

  // x_i: For hard constraint, just the target position because
  //      f_i is the indicator fn.
  math::RealField3 dual_;
  math::RealField3 gap_;
  Real initial_rho_ = 1e4;
};

}  // namespace ax::xpbd
