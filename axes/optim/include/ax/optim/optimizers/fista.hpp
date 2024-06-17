#pragma once
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class Fista final : public OptimizerBase {
public:
  Fista() = default;
  ~Fista() = default;

  OptResult Optimize(const OptProblem &problem, const math::vecxr &x0) const final;

  OptimizerKind GetKind() const final { return OptimizerKind::kFista; }

  void SetMonotonic(bool monotonic) { monotonic_ = monotonic; }

  mutable std::vector<real> tk_;

private:
  real lr_{1};
  real shrink_rate_{0.8};

  bool monotonic_{false};
};

}  // namespace ax::optim
