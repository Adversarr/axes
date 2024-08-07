#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_FullPivHouseHolderQR : public DenseSolverBase {
public:
  math::vecxr Solve(const math::vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kFullPivHouseHolderQR; }

private:
  Eigen::FullPivHouseholderQR<matxxr> impl_;
};

}  // namespace ax::math
