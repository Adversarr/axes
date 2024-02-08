#pragma once
#include "axes/optim/linesearch/linesearch.hpp"
#include "axes/optim/optimizer_base.hpp"

namespace ax::optim {

class Lbfgs : public OptimizerBase {
public:
  explicit Lbfgs();

  ~Lbfgs() override = default;

  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) override;

  void SetOptions(utils::Opt const& options) override;
  
  utils::Opt GetOptions() const override;

  std::function<math::vecxr(math::vecxr const&, math::vecxr const&)>
      central_hessian_;

  idx history_size_{10};

  std::string linesearch_name_;
  utils::uptr<LineSearchBase> linesearch_;
};
}  // namespace ax::optim
