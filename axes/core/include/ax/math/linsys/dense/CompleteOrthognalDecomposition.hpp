#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_CompleteOrthognalDecomposition : public DenseSolverBase {
public:
  RealVectorX Solve(const RealVectorX &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kCompleteOrthognalDecomposition; }

private:
  Eigen::CompleteOrthogonalDecomposition<matxxr> impl_;
};

}  // namespace ax::math
