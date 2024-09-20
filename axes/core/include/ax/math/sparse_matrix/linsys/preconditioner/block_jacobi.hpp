#pragma once
#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

class GeneralSparsePreconditioner_BlockJacobi final : public GeneralSparsePreconditionerBase {
public:
  GeneralSparsePreconditioner_BlockJacobi() = default;
  virtual ~GeneralSparsePreconditioner_BlockJacobi() = default;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;
  void AnalyzePattern() override;
  void Factorize() override;

  GeneralPreconditionerKind GetKind() const override { return GeneralPreconditionerKind::BlockJacobi; }

  BufferPtr<Real> inv_diag_;  // [bs, bs, rows]
};

}  // namespace ax::math