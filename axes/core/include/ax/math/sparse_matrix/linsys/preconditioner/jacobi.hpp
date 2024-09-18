#pragma once
#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

class GeneralSparsePreconditioner_Jacobi final : public GeneralSparsePreconditionerBase {
public:
  GeneralSparsePreconditioner_Jacobi() = default;
  virtual ~GeneralSparsePreconditioner_Jacobi() = default;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;
  void AnalyzePattern() override;
  void Factorize() override;

  BlockPreconditionerKind GetKind() const override { return BlockPreconditionerKind::Jacobi; }

  BufferPtr<Real> inv_diag_;
};

}  // namespace ax::math