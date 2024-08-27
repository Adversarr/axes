#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseCholesky>

namespace ax::math {

class SparseSolver_LDLT : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::LDLT; }

  Eigen::SimplicialLDLT<RealSparseMatrix> solver_;
};
}  // namespace ax::math
