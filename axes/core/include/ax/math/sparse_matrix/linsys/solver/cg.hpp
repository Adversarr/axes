#pragma once
#include "ax/math/sparse_matrix/linsys/solver.hpp"

namespace ax::math {

class GeneralSparseSolver_ConjugateGradient final : public GeneralSparseSolverBase {
public:
  GeneralSparseSolver_ConjugateGradient() = default;
  virtual ~GeneralSparseSolver_ConjugateGradient() = default;

  void AnalyzePattern() override;

  BlockedLinsysSolveStatus Solve(ConstRealBufferView b, RealBufferView x) const override;

  GeneralSparseSolverKind GetKind() const override { return GeneralSparseSolverKind::ConjugateGradient; }
protected:
  BufferPtr<Real> p_buf;
  BufferPtr<Real> d_buf;
  BufferPtr<Real> q_buf;
  BufferPtr<Real> residual_buf;
};

}  // namespace ax::math