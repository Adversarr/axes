
#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_JacobiSVD : public DenseSolverBase {
public:
  RealVectorX Solve(const RealVectorX &b) override;
  void Compute() override;

  DenseSolverKind GetKind() const final { return DenseSolverKind::kJacobiSVD; }

private:
  Eigen::JacobiSVD<matxxr> impl_;
};

}  // namespace ax::math
