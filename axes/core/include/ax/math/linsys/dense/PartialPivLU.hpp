#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_PartialPivLU : public DenseSolverBase {
public:
  vecxr Solve(const vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kPartialPivLU; }

private:
  Eigen::PartialPivLU<matxxr> impl_;
};

}  // namespace ax::math
