#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_CompleteOrthognalDecomposition : public DenseSolverBase {
public:
  math::vecxr Solve(const math::vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kCompleteOrthognalDecomposition; }

private:
  Eigen::CompleteOrthogonalDecomposition<matxxr> impl_;
};

}  // namespace ax::math
