#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_FullPivLU : public DenseSolverBase {
public:
  math::vecxr Solve(const math::vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kFullPivLU; }

private:
  Eigen::FullPivLU<matxxr> impl_;
};
}  // namespace ax::math
