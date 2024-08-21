#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_LeastSquaresConjugateGradient : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kLeastSquaresConjugateGradient; }

private:
  Eigen::LeastSquaresConjugateGradient<RealSparseMatrix> solver_;
};

}  // namespace ax::math
