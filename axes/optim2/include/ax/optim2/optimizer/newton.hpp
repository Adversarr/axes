#pragma once
#include "ax/math/sparse_matrix/linsys/solver.hpp"
#include "base.hpp"

namespace ax::optim2 {

class Optimizer_Newton : public OptimizerBase {
public:
  Optimizer_Newton();

  ~Optimizer_Newton() override;

  OptimizeResult Optimize(OptimizeParam param) override;

  OptimizerKind GetKind() const override;

private:
  std::unique_ptr<math::GeneralSparseSolverBase> linear_solver_;
  BufferPtr<Real> search_direction_;
};

}  // namespace ax::optim2