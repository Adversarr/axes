#pragma once
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

using LbfgsHessianApproximator
    = std::function<math::vecxr(Gradient const& /*r, solve this*/,
                                Variable const& /*x_k+1 - x_k, if possible else empty*/,
                                Variable const& /*g_k+1 - g_k, if possible else empty*/)>;

class Optimizer_Lbfgs : public OptimizerBase {
public:
  explicit Optimizer_Lbfgs();

  ~Optimizer_Lbfgs() override = default;

  OptResult Optimize(OptProblem const& problem, const Variable& x0) const override;

  OptimizerKind GetKind() const override { return OptimizerKind::kLbfgs; }

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const override;

  idx history_size_{20};

  void SetApproxSolve(LbfgsHessianApproximator approximator);

protected:
  bool check_approx_quality_{false};
  std::unique_ptr<LinesearchBase> linesearch_;
  LbfgsHessianApproximator approx_solve_;

};

}  // namespace ax::optim
