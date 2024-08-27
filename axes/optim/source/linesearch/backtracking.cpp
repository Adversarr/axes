#include "ax/optim/linesearch/backtracking.hpp"

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/optim/linesearch/linesearch.hpp"

namespace ax::optim {

OptResult Linesearch_Backtracking::Optimize(OptProblem const& prob, Variable const& x0,
                                            Gradient const& grad, Variable const& dir) const {
  // SECT: Check Inputs
  AX_THROW_IF_LT(initial_step_size_, 0, "Initial step length must be positive");
  AX_THROW_IF_TRUE(required_descent_rate_ <= 0 || required_descent_rate_ >= 1,
                   "Required descent rate must be in (0, 1)");
  AX_THROW_IF_FALSE(prob.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(prob.HasEnergy(), "Energy function not set");

  if (step_shrink_rate_ <= 0) {
    AX_THROW_IF_TRUE(min_step_shrink_rate_ <= 0 || min_step_shrink_rate_ >= max_step_shrink_rate_
                         || max_step_shrink_rate_ >= 1,
                     "Invalid adaptive step size parameters {} {}", min_step_shrink_rate_,
                     max_step_shrink_rate_);
  } else {
    AX_THROW_IF_TRUE(step_shrink_rate_ >= 1, "Invalid fixed step size parameters {}",
                     step_shrink_rate_);
  }

  // SECT: Backtracking Line Search
  Real alpha = initial_step_size_;
  Real const f0 = prob.EvalEnergy(x0);
  AX_THROW_IF_FALSE(math::isfinite(f0),
                    "Invalid x0 in Line Search, Energy returns infinite number.");

  Real expected_descent = math::dot(grad, dir);  // Also the directional derivative at 0.
  AX_THROW_IF_TRUE(expected_descent >= 0 || !math::isfinite(expected_descent),
                   "Invalid descent direction: Dot[grad, dir]={}", expected_descent);

  Index iter = 0;
  Variable x = x0;
  Real f = math::inf<Real>;
  bool success = false;

  for (; iter < max_iter_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    if (prob.HasProximator()) {  // If proximator is available, apply it
      x = prob.EvalProximator(x, alpha);
      expected_descent = math::dot(grad, x - x0);
    }

    // Evaluate the energy at the new point
    f = prob.EvalEnergy(x);
    if (examine_arjimo_condition(f, f0, alpha * expected_descent, required_descent_rate_)
        && math::isfinite(f)) {  // Satisfies the Armijo condition
      success = true;
      break;
    }

    // Shrink the step size
    Real shrink_rate = step_shrink_rate_;  // Runtime determine the shrink rate
    if (shrink_rate <= 0) {                // if the adaptive mode is enabled
      // Fit a quadratic model to make a better guess of the step size
      // phi(x) = c2 x^2 + c1 x + c0 => phi(alpha) = f0 + c1 alpha + c2 alpha^2
      Real c0 = f0;
      Real c1 = expected_descent;
      Real c2 = (f - c0 - c1 * alpha) / (alpha * alpha);
      // The optimal step size is -c1 / (2 * c2)
      shrink_rate = -c1 / (2 * c2);
      if (!math::isfinite(shrink_rate)) {
        shrink_rate = 0.5 * (min_step_shrink_rate_ + max_step_shrink_rate_);
      } else {
        shrink_rate = math::clamp(shrink_rate, min_step_shrink_rate_, max_step_shrink_rate_);
      }
      AX_DEBUG("Adaptive shrink rate: {:12.6e} in [{:12.6e} {:12.6e}]", shrink_rate,
               min_step_shrink_rate_, max_step_shrink_rate_);
    }
    alpha *= shrink_rate;
  }

  if (success) {
    return OptResult::ConvergedLinesearch(std::move(x), f, iter, alpha);
  }

  OptResult opt;
  opt.x_opt_ = std::move(x);
  opt.f_opt_ = f;
  opt.n_iter_ = iter;
  opt.converged_ = false;
  opt.step_length_ = alpha;
  return opt;
}

void Linesearch_Backtracking::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT(options, Real, step_shrink_rate);
  AX_SYNC_OPT(options, Real, required_descent_rate);
  AX_SYNC_OPT(options, Real, min_step_shrink_rate);
  AX_SYNC_OPT(options, Real, max_step_shrink_rate);
  LinesearchBase::SetOptions(options);
}

utils::Options Linesearch_Backtracking::GetOptions() const {
  utils::Options opt = LinesearchBase::GetOptions();
  opt["step_shrink_rate"] = step_shrink_rate_;
  opt["required_descent_rate"] = required_descent_rate_;
  opt["min_step_shrink_rate"] = min_step_shrink_rate_;
  opt["max_step_shrink_rate"] = max_step_shrink_rate_;
  return opt;
}

LineSearchKind Linesearch_Backtracking::GetKind() const {
  return LineSearchKind::Backtracking;
}
}  // namespace ax::optim
