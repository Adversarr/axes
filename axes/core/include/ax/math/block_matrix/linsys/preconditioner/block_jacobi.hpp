#pragma once
#include "ax/math/block_matrix/linsys/preconditioner.hpp"

namespace ax::math {

class BlockPreconditioner_BlockJacobi final : public BlockPreconditionerBase {
public:
  BlockPreconditioner_BlockJacobi() = default;
  virtual ~BlockPreconditioner_BlockJacobi() = default;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;
  void AnalyzePattern() override;
  void Factorize() override;

  BlockPreconditionerKind GetKind() const override { return BlockPreconditionerKind::BlockJacobi; }

private:
  BufferPtr<Real> inv_diag_; // [bs, bs, rows]
};

}  // namespace ax::math