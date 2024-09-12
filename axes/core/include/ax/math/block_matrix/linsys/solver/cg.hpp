#pragma once
#include "ax/math/block_matrix/linsys/solver.hpp"

namespace ax::math {

class BlockSolver_ConjugateGradient final : public BlockSolverBase {
public:
  BlockSolver_ConjugateGradient() = default;
  virtual ~BlockSolver_ConjugateGradient() = default;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;
  void AnalyzePattern() override;
  void Factorize() override;

  BlockSolverKind GetKind() const override { return BlockSolverKind::ConjugateGradient; }
};

}  // namespace ax::math