#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_ConjugateGradient : public HostSparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  void SetOptions(utils::Options const &) final;

  utils::Options GetOptions() const final;

  HostSparseSolverKind GetKind() const final { return HostSparseSolverKind::ConjugateGradient; }

private:
  Eigen::ConjugateGradient<RealSparseMatrix, Eigen::Lower | Eigen::Upper,
                           Eigen::IncompleteCholesky<Real, Eigen::Lower, Eigen::AMDOrdering<SparseIndex>>>
      solver_;

  Index max_iter_ = 100;
  Real tol_ = 1e-6;
};
}  // namespace ax::math
