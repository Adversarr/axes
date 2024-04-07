#pragma once
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::optim {

class Lbfgs : public OptimizerBase {
public:
  explicit Lbfgs();

  ~Lbfgs() override = default;

  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  Status SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const override;

  std::function<math::vecxr(math::vecxr const&, math::vecxr const&)> central_hessian_;

  idx history_size_{3};

  void SetApproxSolve(std::function<math::vecxr(math::vecxr const&)> hessian_approximation);

protected:
  std::string linesearch_name_;
  UPtr<LinesearchBase> linesearch_;
  std::function<math::vecxr(math::vecxr const& )> approx_solve_;
};
}  // namespace ax::optim
