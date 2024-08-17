#pragma once
#include "ax/xpbd/common.hpp"

namespace ax::xpbd {

class Constraint_Tetra final : public ConstraintBase {
public:
  ConstraintKind GetKind() const override { return ConstraintKind::kTetra; }

  ConstraintSolution SolveDistributed() override;
  void BeginStep() override;
  real UpdateDuality() override;
  void EndStep() override;
  void UpdateRhoConsensus(real scale) override;
  void SetTetrahedrons(math::IndexField4 const& tetrahedrons, math::RealField1 const& stiff);

  math::RealField1 stiffness_;
  std::vector<math::RealMatrix<3, 4>> dual_;
  std::vector<math::RealMatrix<3, 4>> gap_;
  std::vector<math::RealMatrix<3, 4>> x0_;
  std::vector<math::RealMatrix<3, 3>> rotation_;
  Index substeps_{2};
};

}  // namespace ax::xpbd
