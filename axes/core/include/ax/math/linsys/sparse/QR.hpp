#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseQR>

namespace ax::math {

class SparseSolver_QR : public HostSparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  HostSparseSolverKind GetKind() const final { return HostSparseSolverKind::QR; }

private:
  Eigen::SparseQR<RealSparseMatrix, Eigen::COLAMDOrdering<SparseIndex>> solver_;
};

}  // namespace ax::math
