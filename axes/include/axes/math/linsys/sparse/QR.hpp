#pragma once
#include "axes/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_QR : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

  SparseSolverKind Kind() const final { return SparseSolverKind::kQR; }

private:
  Eigen::SparseQR<sp_matxxr, Eigen::COLAMDOrdering<idx>> solver_;
};

}  // namespace ax::math