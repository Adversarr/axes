#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_HouseholderQR : public DenseSolverBase {
public:
  RealVectorX Solve(const RealVectorX &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kHouseholderQR; }

private:
  Eigen::ColPivHouseholderQR<matxxr> impl_;
};
}  // namespace ax::math
