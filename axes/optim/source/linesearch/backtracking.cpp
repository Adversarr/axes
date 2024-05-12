#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
namespace ax::optim {
OptResult Linesearch_Backtracking::Optimize(OptProblem const& prob, math::vecxr const& x0,
                                           math::vecxr const& grad, math::vecxr const& dir) const {
  // SECT: Check Inputs
  if (initial_step_length_ <= 0) {
    return utils::InvalidArgumentError("Invalid alpha_: " + std::to_string(initial_step_length_));
  }
  if (step_shrink_rate_ <= 0 || step_shrink_rate_ >= 1) {
    return utils::InvalidArgumentError("Invalid rho_: " + std::to_string(step_shrink_rate_));
  }
  if (required_descent_rate_ <= 0 || required_descent_rate_ > 1) {
    return utils::InvalidArgumentError("Invalid c_: " + std::to_string(required_descent_rate_));
  }
  if (!prob.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }
  if (!prob.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  }

  // SECT: Backtracking Line Search
  real alpha = initial_step_length_;
  real const f0 = prob.EvalEnergy(x0);
  if (!math::isfinite(f0)) {
    return utils::FailedPreconditionError("Invalid x0 in Line Search, Energy returns infinite number.");
  }
  real expected_descent = grad.dot(dir);
  if (expected_descent >= 0 || !math::isfinite(expected_descent)) {
    AX_LOG(ERROR) << "grad: " << grad.transpose();
    AX_LOG(ERROR) << "dir: " << dir.transpose();
    return utils::FailedPreconditionError("Invalid descent direction: df0=" + std::to_string(expected_descent));
  }

  idx iter = 0;
  math::vecxr x;
  real f;
  for (; iter < max_iter_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    f = prob.EvalEnergy(x);
    if (examine_arjimo_condition(f, f0, expected_descent, required_descent_rate_, alpha) && math::isfinite(f)) {
      break;
    }
    alpha *= step_shrink_rate_;
  }

  OptResultImpl opt;
  opt.x_opt_ = x;
  opt.f_opt_ = f;
  opt.n_iter_ = iter;
  opt.converged_ = iter < max_iter_;
  opt.step_length_ = alpha;
  return opt;
}

Status Linesearch_Backtracking::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT(options, real, initial_step_length);
  AX_SYNC_OPT(options, real, step_shrink_rate);
  AX_SYNC_OPT(options, real, required_descent_rate);
  return LinesearchBase::SetOptions(options);
}

utils::Opt Linesearch_Backtracking::GetOptions() const {
  utils::Opt opt = LinesearchBase::GetOptions();
  opt["initial_step_length"] = initial_step_length_;
  opt["step_shrink_rate"] = step_shrink_rate_;
  opt["required_descent_rate"] = required_descent_rate_;
  return opt;
}

LineSearchKind Linesearch_Backtracking::GetKind() const { return LineSearchKind::kBacktracking; }
}  // namespace ax::optim
