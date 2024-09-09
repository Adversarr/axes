#pragma once
#include "ax/math/linsys/sparse.hpp"
namespace ax::math {

class SparseSolver_Jacobi final : public SparseSolverBase {
public:
  SparseSolver_Jacobi() = default;
  ~SparseSolver_Jacobi() = default;

  LinsysSolveResult Solve(const RealMatrixX &b, const RealMatrixX &x0 = {}) override;

  void AnalyzePattern() override {}
  void Factorize() override {}

  SparseSolverKind GetKind() const override;
};

}