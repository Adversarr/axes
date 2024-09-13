#pragma once
#include "ax/math/block_matrix/linsys/preconditioner.hpp"

namespace ax::math {

class BlockPreconditioner_Jacobi final : public BlockPreconditionerBase {
public:
  BlockPreconditioner_Jacobi() = default;
  virtual ~BlockPreconditioner_Jacobi() = default;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;
  void AnalyzePattern() override;
  void Factorize() override;

  BlockPreconditionerKind GetKind() const override { return BlockPreconditionerKind::Jacobi; }

  BufferPtr<Real> inv_diag_;
};

}  // namespace ax::math