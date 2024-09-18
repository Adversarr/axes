#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseLU>

namespace ax::math {

class SparseSolver_LU : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  HostSparseSolverKind GetKind() const final { return HostSparseSolverKind::LU; }

private:
  Eigen::SparseLU<RealSparseMatrix, Eigen::COLAMDOrdering<Index>> solver_;
};
}  // namespace ax::math
