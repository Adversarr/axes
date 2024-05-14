#include "ax/optim/linesearch/wolfe.hpp"

#include "ax/core/excepts.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/opt.hpp"
namespace ax::optim {
OptResult Linesearch_Wofle::Optimize(OptProblem const& prob, math::vecxr const& x0,
                                     math::vecxr const& grad, math::vecxr const& dir) const {
  // SECT: Check Inputs
  // if (initial_step_length_ <= 0) {
  //   return utils::InvalidArgumentError("Invalid alpha_: " + std::to_string(initial_step_length_));
  // }
  // if (step_shrink_rate_ <= 0 || step_shrink_rate_ >= 1) {
  //   return utils::InvalidArgumentError("Invalid rho_: " + std::to_string(step_shrink_rate_));
  // }
  // if (required_descent_rate_ <= 0 || required_descent_rate_ > 1) {
  //   return utils::InvalidArgumentError("Invalid c_: " + std::to_string(required_descent_rate_));
  // }
  // if (!prob.HasEnergy()) {
  //   return utils::FailedPreconditionError("Energy function not set");
  // }
  // if (!prob.HasGrad()) {
  //   return utils::FailedPreconditionError("Gradient function not set");
  // }

  AX_THROW_IF_LT(initial_step_length_, 0, "Initial step length must be positive");
  AX_THROW_IF_FALSE(0 < step_shrink_rate_ && step_shrink_rate_ < 1, "Step shrink rate must be in (0, 1)");
  AX_THROW_IF_FALSE(0 < required_descent_rate_ && required_descent_rate_ < 1, "Required descent rate must be in (0, 1)");
  AX_THROW_IF_FALSE(prob.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(prob.HasEnergy(), "Energy function not set");


  // SECT: Backtracking Line Search
  real alpha = initial_step_length_;
  real const f0 = prob.EvalEnergy(x0);
  // if (!math::isfinite(f0)) {
  //   return utils::FailedPreconditionError(
  //       "Invalid x0 in Line Search, Energy returns infinite number.");
  // }
  AX_THROW_IF_FALSE(math::isfinite(f0), "Invalid x0 in Line Search, Energy returns infinite number.");
  real const expected_descent = grad.dot(dir);
  // if (expected_descent >= 0 || !math::isfinite(expected_descent)) {
  //   AX_LOG(ERROR) << "grad: " << grad.transpose();
  //   AX_LOG(ERROR) << "dir: " << dir.transpose();
  //   return utils::FailedPreconditionError("Invalid descent direction: df0="
  //                                         + std::to_string(expected_descent));
  // }
  AX_THROW_IF_FALSE(expected_descent >= 0 || !math::isfinite(expected_descent), "Invalid descent direction: df0="
                                                                            + std::to_string(expected_descent));
  idx iter = 0;
  math::vecxr g;
  math::vecxr x;
  real f = f0;
  for (; iter < max_iter_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    g = prob.EvalGrad(x);
    f = prob.EvalEnergy(x);
    if (examine_arjimo_condition(f, f0, expected_descent, required_descent_rate_, alpha)
        && examine_curvature_condition(dir, g, expected_descent, required_curvature_rate_)
        && (!strong_wolfe_
            || examine_strong_wolfe_condition(dir, g, expected_descent, required_curvature_rate_))
        && math::isfinite(f)) {
      break;
    }
    alpha *= step_shrink_rate_;
  }

  OptResult opt;
  opt.x_opt_ = x;
  opt.f_opt_ = f;
  opt.n_iter_ = iter;
  opt.converged_ = iter < max_iter_;
  opt.step_length_ = alpha;
  return opt;
}

void Linesearch_Wofle::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT(options, real, initial_step_length);
  AX_SYNC_OPT(options, real, step_shrink_rate);
  AX_SYNC_OPT(options, real, required_descent_rate);
  AX_SYNC_OPT(options, real, required_curvature_rate);
  AX_SYNC_OPT(options, bool, strong_wolfe);
  LinesearchBase::SetOptions(options);
}

utils::Opt Linesearch_Wofle::GetOptions() const {
  utils::Opt opt = LinesearchBase::GetOptions();
  opt["initial_step_length"] = initial_step_length_;
  opt["step_shrink_rate"] = step_shrink_rate_;
  opt["required_descent_rate"] = required_descent_rate_;
  opt["strong_wolfe"] = strong_wolfe_;
  return opt;
}

LineSearchKind Linesearch_Wofle::GetKind() const { return LineSearchKind::kWolfe; }
}  // namespace ax::optim
