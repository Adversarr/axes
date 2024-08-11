#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseLU>

namespace ax::math {

class SparseSolver_LU : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(matxxr const &b, matxxr const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kLU; }

private:
  Eigen::SparseLU<spmatr, Eigen::COLAMDOrdering<idx>> solver_;
};
}  // namespace ax::math
