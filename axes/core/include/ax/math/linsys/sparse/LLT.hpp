#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseCholesky>

namespace ax::math {

class SparseSolver_LLT : public HostSparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(RealMatrixX const &b, RealMatrixX const &x0) override;

  HostSparseSolverKind GetKind() const final { return HostSparseSolverKind::LLT; }

private:
  Eigen::SimplicialLLT<RealSparseMatrix> solver_;
};

}  // namespace ax::math
