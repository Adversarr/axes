#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_LLT : public DenseSolverBase {
public:
  math::vecxr Solve(const math::vecxr &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kLLT; }

private:
  Eigen::LLT<matxxr> impl_;
};
}  // namespace ax::math
