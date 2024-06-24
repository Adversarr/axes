#pragma once
#include "ax/math/linsys/sparse.hpp"
#include <Eigen/SparseCholesky>

namespace ax::math {

class SparseSolver_LLT : public SparseSolverBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  LinsysSolveResult Solve(vecxr const &b, vecxr const &x0) override;

  SparseSolverKind GetKind() const final { return SparseSolverKind::kLLT; }

private:
  Eigen::SimplicialLLT<spmatr> solver_;
};

}  // namespace ax::math
