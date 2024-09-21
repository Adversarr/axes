#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/IterativeLinearSolvers>

namespace ax::math {

class SparseSolver_BiCGSTAB : public HostSparseSolverBase {
public:
  void AnalyzePattern() final;

  void Factorize() final;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  HostSparseSolverKind GetKind() const final { return HostSparseSolverKind::BiCGSTAB; }

private:
  Eigen::BiCGSTAB<RealSparseMatrix> solver_;
};
}  // namespace ax::math
