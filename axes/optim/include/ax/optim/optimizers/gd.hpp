#pragma once
#include "ax/core/common.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class GradientDescent : public OptimizerBase {
public:
  GradientDescent(real const& lr = 0.01) : lr_(lr) {}

  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  void SetLearningRate(real const& lr);

  void SetProximator(std::function<math::vecxr(math::vecxr const&, real)> proximator);

  void SetLineSearch(UPtr<LinesearchBase> linesearch);

  void EnableFista(bool enable) { enable_fista_ = enable; }

private:
  UPtr<LinesearchBase> linesearch_;
  std::function<math::vecxr(math::vecxr const&, real)> proximator_;
  real lr_;
  bool enable_fista_ = false;
};

}  // namespace ax::optim
