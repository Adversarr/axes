#pragma once
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

using LbfgsHessianApproximator
    = std::function<math::vecxr(math::vecxr const& /*r, solve this*/,
                                math::vecxr const& /*x_k+1 - x_k, if possible else empty*/,
                                math::vecxr const& /*g_k+1 - g_k, if possible else empty*/)>;

class Lbfgs : public OptimizerBase {
public:
  explicit Lbfgs();

  ~Lbfgs() override = default;

  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  OptimizerKind GetKind() const override { return OptimizerKind::kLbfgs; }

  void SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const override;

  idx history_size_{20};

  void SetApproxSolve(LbfgsHessianApproximator approximator);

protected:
  std::string linesearch_name_;
  bool check_approx_quality_{false};
  UPtr<LinesearchBase> linesearch_;
  LbfgsHessianApproximator approx_solve_;

};
}  // namespace ax::optim
