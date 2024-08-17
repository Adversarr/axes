#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseQR>

namespace ax::math {

class SparseSolver_QR : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(matxxr const &b, matxxr const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kQR; }

private:
  Eigen::SparseQR<spmatr, Eigen::COLAMDOrdering<Index>> solver_;
};

}  // namespace ax::math
