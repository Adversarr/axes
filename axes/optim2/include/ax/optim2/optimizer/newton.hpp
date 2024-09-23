#pragma once
#include "ax/math/sparse_matrix/linsys/solver.hpp"
#include "base.hpp"

namespace ax::optim2 {

class Optimizer_Newton : public OptimizerBase {
public:
  Optimizer_Newton();

  void Optimize() const override;

  OptimizerKind GetKind() const override;

private:
  std::unique_ptr<math::GeneralSparseSolverBase> linear_solver_;
};

}  // namespace ax::optim2