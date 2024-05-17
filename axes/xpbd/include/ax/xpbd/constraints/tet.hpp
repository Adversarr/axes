#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

template <idx dim> class Constraint_Tetra final : public ConstraintBase<dim> {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kTetra; }

  ConstraintSolution<dim> SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void SetTetrahedrons(math::fieldi<dim + 1> const& tetrahedrons, math::field1r const& stiff);

  math::field1r stiffness_;
  std::vector<math::matr<dim, dim + 1>> dual_;
  std::vector<math::matr<dim, dim + 1>> gap_;
  std::vector<math::matr<dim, dim + 1>> x0_;
  std::vector<math::matr<dim, dim>> rotation_;
  idx substeps_{2};
};

}  // namespace ax::xpbd
