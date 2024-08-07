#pragma once
#include "ax/core/common.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class Optimizer_GradientDescent : public OptimizerBase {
public:
  Optimizer_GradientDescent(real const& lr = 0.01) : lr_(lr) {}

  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  OptimizerKind GetKind() const override;

  void SetLearningRate(real const& lr);

  void SetProximator(std::function<math::vecxr(math::vecxr const&, real)> proximator);

  void SetLineSearch(UPtr<LinesearchBase> linesearch);

  void SetOptions(const utils::Options &options) override;

  utils::Options GetOptions() const override;

private:
  UPtr<LinesearchBase> linesearch_;
  real lr_;
};

}  // namespace ax::optim
