
#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_LDLT final : public DenseSolverBase {
public:
  vecxr Solve(const vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kLDLT; }

private:
  Eigen::LDLT<matxxr> impl_;
};
}  // namespace ax::math
