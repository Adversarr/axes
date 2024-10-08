#pragma once

#include "ax/xpbd/common.hpp"

namespace ax::xpbd {
class Constraint_Spring final : public ConstraintBase {
public:
  Constraint_Spring() = default;

  void SetSprings(math::IndexField2 const& indices, math::RealField1 const& stiffness);

  ConstraintKind GetKind() const override { return ConstraintKind::Spring; }

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  Real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(Real scale) override;


  math::RealField1 spring_stiffness_;
  math::RealField1 spring_length_;

  // x_i
  math::RealField<6> dual_;
  math::RealField<6> gap_;
};
}  // namespace ax::xpbd
