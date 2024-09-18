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

  BlockPreconditionerKind GetKind() const override { return BlockPreconditionerKind::BlockJacobi; }

  BufferPtr<Real> inv_diag_;  // [bs, bs, rows]
};

}  // namespace ax::math