#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseQR>

namespace ax::math {

class SparseSolver_QR : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(vecxr const &b, vecxr const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kQR; }

private:
  Eigen::SparseQR<spmatr, Eigen::COLAMDOrdering<idx>> solver_;
};

}  // namespace ax::math
