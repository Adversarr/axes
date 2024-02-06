#include "axes/optim/linesearch/backtracking.hpp"
namespace ax::optim {
OptResult BacktrackingLinesearch::Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& dir,
                                           utils::Opt const& options) {
  real alpha = options.Get<real>("alpha", alpha_);
  real rho = options.Get<real>("beta", rho_);
  real c = options.Get<real>("c", c_);

  // SECT: Check Inputs
  if (alpha <= 0) {
    return utils::InvalidArgumentError("Invalid alpha: " + std::to_string(alpha));
  }
  if (rho <= 0 || rho > 1) {
    return utils::InvalidArgumentError("Invalid rho: " + std::to_string(rho));
  }
  if (c <= 0 || c > 1) {
    return utils::InvalidArgumentError("Invalid c: " + std::to_string(c));
  }
  if (!prob.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }

  if (!prob.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  }

  // SECT: Backtracking Line Search
  real f0 = prob.EvalEnergy(x0);
  real df0 = prob.EvalGrad(x0).dot(dir);
  if (df0 >= 0) {
    return utils::InvalidArgumentError("Invalid descent direction: df0=" + std::to_string(df0));
  }
  idx iter = 0;

  OptResultImpl opt;
  while (true) {
    math::vecxr x = x0 + alpha * dir;
    real f = prob.EvalEnergy(x);
    if (f <= f0 + c * alpha * df0) {
      opt = OptResultImpl{x, f, iter};
      opt.converged_ = true;
      break;
    }
    alpha *= rho;
    iter++;
    if (iter > max_iter_) {
      opt = OptResultImpl{x, f, iter};
      opt.converged_ = false;
      break;
    }
  }
  DLOG(INFO) << "BTLS[f0=" << f0 << ", df0=" << df0 << ", it=" << iter << "]";
  return opt;
}
}  // namespace ax::optim