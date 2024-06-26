#pragma once
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class Lbfgs : public OptimizerBase {
public:
  explicit Lbfgs();

  ~Lbfgs() override = default;

  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  OptimizerKind GetKind() const override { return OptimizerKind::kLbfgs; }

  void SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const override;

  std::function<math::vecxr(math::vecxr const&, math::vecxr const&)> central_hessian_;

  idx history_size_{20};

  void SetApproxSolve(std::function<math::vecxr(math::vecxr const&)> hessian_approximation);

protected:
  std::string linesearch_name_;
  bool check_approx_quality_{false};
  UPtr<LinesearchBase> linesearch_;
  std::function<math::vecxr(math::vecxr const& )> approx_solve_;
};
}  // namespace ax::optim
