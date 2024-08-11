#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_LeastSquaresConjugateGradient : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(matxxr const &b, matxxr const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kLeastSquaresConjugateGradient; }

private:
  Eigen::LeastSquaresConjugateGradient<spmatr> solver_;
};

}  // namespace ax::math
