#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_ConjugateGradient : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(matxxr const &b, matxxr const &x0) override;

  void SetOptions(utils::Options const &) final;

  utils::Options GetOptions() const final;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kConjugateGradient; }

private:
  Eigen::ConjugateGradient<spmatr, Eigen::Lower | Eigen::Upper,
                           Eigen::IncompleteCholesky<real, Eigen::Lower, Eigen::AMDOrdering<Index>>>
      solver_;

  Index max_iter_ = 100;
  real tol_ = 1e-6;
};
}  // namespace ax::math
