#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_BiCGSTAB : public SparseSolverBase {
public:
  void AnalyzePattern() final;

  void Factorize() final;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kBiCGSTAB; }

private:
  Eigen::BiCGSTAB<RealSparseMatrix> solver_;
};
}  // namespace ax::math
