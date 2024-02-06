#pragma once
#include "axes/optim/optimizer_base.hpp"

namespace ax::optim {

class Lbfgs : public OptimizerBase {
public:
  explicit Lbfgs(OptProblem& problem) : OptimizerBase{problem} {}

  ~Lbfgs() override = default;

  OptResult Optimize(math::vecxr const& x0, utils::Opt const& options) override;

  std::function<math::vecxr(math::vecxr const&, math::vecxr const&)>
      central_hessian_;

  idx history_size_{10};
};
}  // namespace ax::optim
