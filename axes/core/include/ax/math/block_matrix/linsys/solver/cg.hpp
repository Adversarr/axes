#pragma once
#include "ax/math/block_matrix/linsys/solver.hpp"

namespace ax::math {

class BlockSolver_ConjugateGradient final : public BlockSolverBase {
public:
  BlockSolver_ConjugateGradient() = default;
  virtual ~BlockSolver_ConjugateGradient() = default;

  BlockedLinsysSolveStatus Solve(ConstRealBufferView b, RealBufferView x) const override;
  void AnalyzePattern() override;
  void Factorize() override;

  BlockSolverKind GetKind() const override { return BlockSolverKind::ConjugateGradient; }
  size_t max_iter_{1000};
};

}  // namespace ax::math