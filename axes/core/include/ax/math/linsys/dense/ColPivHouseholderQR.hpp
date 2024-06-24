#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_ColPivHouseholderQR : public DenseSolverBase {
public:
  math::vecxr Solve(const math::vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kColPivHouseholderQR; }
private:
  Eigen::ColPivHouseholderQR<matxxr> impl_;
};

}  // namespace ax::math
