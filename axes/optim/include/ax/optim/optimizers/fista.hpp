#pragma once
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class Optimizer_Fista final : public OptimizerBase {
public:
  Optimizer_Fista() = default;
  ~Optimizer_Fista() = default;

  OptResult Optimize(const OptProblem &problem, const Variable &x0) const final;

  OptimizerKind GetKind() const final { return OptimizerKind::kFista; }

  void SetMonotonic(bool monotonic) { monotonic_ = monotonic; }

  mutable std::vector<Real> tk_;

private:
  Real lr_{1};
  Real shrink_rate_{0.8};

  bool monotonic_{false};
};

}  // namespace ax::optim
