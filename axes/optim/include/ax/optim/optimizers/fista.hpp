#pragma once
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class Optimizer_Fista final : public OptimizerBase {
public:
  Optimizer_Fista() = default;
  ~Optimizer_Fista() = default;

  OptResult Optimize(const OptProblem &problem, const math::vecxr &x0) const final;

  OptimizerKind GetKind() const final { return OptimizerKind::kFista; }

private:
  real lr_{1};
  real shrink_rate_{0.5};
};

}  // namespace ax::optim
