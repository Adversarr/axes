#include "axes/optim/linesearch/backtracking.hpp"
namespace ax::optim {
OptResult BacktrackingLinesearch::Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& dir,
                                           utils::Opt const& options) {
  real alpha = options.Get<real>("alpha", alpha_);
  real beta = options.Get<real>("beta", beta_);
  if (alpha <= 0 || alpha > 1) {
    return utils::InvalidArgumentError("Invalid alpha: " + std::to_string(alpha));
  }
  if (beta <= 0 || beta > 1) {
    return utils::InvalidArgumentError("Invalid beta: " + std::to_string(beta));
  }

  if (!prob.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }

  if (!prob.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  }

  real f0 = prob.EvalEnergy(x0);
  real df0 = prob.EvalGrad(x0).dot(dir);
  if (df0 >= 0) {
    return utils::InvalidArgumentError("Invalid descent direction: df0=" + std::to_string(df0));
  }
  real alpha0 = alpha;
  idx iter = 0;

  OptResultImpl opt;
  while (true) {
    math::vecxr x = x0 + alpha * dir;
    real f = prob.EvalEnergy(x);
    if (f <= f0 + alpha * df0) {
      opt = OptResultImpl{x, f, iter};
      opt.converged_ = true;
      break;
    }
    alpha *= beta;
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