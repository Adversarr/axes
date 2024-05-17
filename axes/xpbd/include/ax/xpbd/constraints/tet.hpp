#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

template<idx dim>
class Constraint_Tetra final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kTetra; }

  ConstraintSolution<dim> SolveDistributed() override;

  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;

  math::field1r stiffness_;
  math::fieldr<dim * (dim + 1)> dual_;
  math::fieldr<dim * (dim + 1)> gap_;
};

}
