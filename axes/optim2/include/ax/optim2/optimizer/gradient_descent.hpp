#pragma once
#include "base.hpp"

namespace ax::optim2 {

class Optimizer_GradientDescent : public OptimizerBase {
public:
  Optimizer_GradientDescent();
  ~Optimizer_GradientDescent() override = default;

  OptimizeResult Optimize(OptimizeParam param) override;

  OptimizerKind GetKind() const override { return OptimizerKind::GradientDescent; }

private:
  BufferPtr<Real> search_direction_;
};

}  // namespace ax::optim2