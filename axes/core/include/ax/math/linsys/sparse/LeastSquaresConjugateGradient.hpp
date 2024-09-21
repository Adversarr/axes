#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_LeastSquaresConjugateGradient : public HostSparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  HostSparseSolverKind GetKind() const final { return HostSparseSolverKind::LeastSquaresConjugateGradient; }

private:
  Eigen::LeastSquaresConjugateGradient<RealSparseMatrix> solver_;
};

}  // namespace ax::math
