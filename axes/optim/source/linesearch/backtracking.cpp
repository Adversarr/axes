#include "ax/optim/linesearch/backtracking.hpp"
namespace ax::optim {
OptResult BacktrackingLinesearch::Optimize(OptProblem const& prob, math::vecxr const& x0,
                                           math::vecxr const& dir) const {
  // SECT: Check Inputs
  if (alpha_ <= 0) {
    return utils::InvalidArgumentError("Invalid alpha_: " + std::to_string(alpha_));
  }
  if (rho_ <= 0 || rho_ >= 1) {
    return utils::InvalidArgumentError("Invalid rho_: " + std::to_string(rho_));
  }
  if (c_ <= 0 || c_ > 1) {
    return utils::InvalidArgumentError("Invalid c_: " + std::to_string(c_));
  }
  if (!prob.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }
  if (!prob.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  }

  // SECT: Backtracking Line Search
  real alpha = alpha_;
  real const f0 = prob.EvalEnergy(x0);
  math::vecxr grad = prob.EvalGrad(x0);
  real df0 = grad.dot(dir);
  if (df0 >= 0 || !math::isfinite(df0)) {
    AX_LOG(ERROR) << "grad: " << grad.transpose();
    AX_LOG(ERROR) << "dir: " << dir.transpose();
    return utils::FailedPreconditionError("Invalid descent direction: df0=" + std::to_string(df0));
  }
  idx iter = 0;
  real expect = f0 + c_ * df0;
  AX_LOG(INFO) << "f0: " << f0 << " df0: " << df0;
  OptResultImpl opt;
  while (true) {
    math::vecxr x = x0 + alpha * dir;
    real const f = prob.EvalEnergy(x);
    expect = f0 + c_ * alpha * df0;
    if (f <= expect) {
      opt = OptResultImpl{x, f, iter};
      opt.converged_ = true;
      break;
    }
    alpha *= rho_;
    std::cout << "LS [" << iter << "]: " << f - expect << " > 0, " << f - f0 << std::endl;
    iter++;
    if (iter > max_iter_) {
      opt = OptResultImpl{x, f, iter};
      opt.converged_ = false;
      break;
    }
  }

  return opt;
}

Status BacktrackingLinesearch::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT(options, real, alpha);
  AX_SYNC_OPT(options, real, rho);
  AX_SYNC_OPT(options, real, c);
  return LinesearchBase::SetOptions(options);
}

utils::Opt BacktrackingLinesearch::GetOptions() const {
  utils::Opt opt = LinesearchBase::GetOptions();
  opt["alpha"] = alpha_;
  opt["rho"] = rho_;
  opt["c"] = c_;
  return opt;
}

}  // namespace ax::optim
