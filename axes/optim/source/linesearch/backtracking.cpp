#include "ax/optim/linesearch/backtracking.hpp"

#include "ax/math/utils/formatting.hpp"
#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
namespace ax::optim {


OptResult Linesearch_Backtracking::Optimize(OptProblem const& prob, Variable const& x0,
                                           Gradient const& grad, Variable const& dir) const {
  // SECT: Check Inputs
  AX_THROW_IF_LT(initial_step_length_, 0, "Initial step length must be positive");
  AX_THROW_IF_FALSE(0 < step_shrink_rate_ && step_shrink_rate_ < 1, "Step shrink rate must be in (0, 1)");
  AX_THROW_IF_FALSE(0 < required_descent_rate_ && required_descent_rate_ < 1, "Required descent rate must be in (0, 1)");
  AX_THROW_IF_FALSE(prob.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(prob.HasEnergy(), "Energy function not set");

  // SECT: Backtracking Line Search
  Real alpha = initial_step_length_;
  Real const f0 = prob.EvalEnergy(x0);
  AX_THROW_IF_FALSE(math::isfinite(f0), "Invalid x0 in Line Search, Energy returns infinite number.");
  Real expected_descent = math::dot(grad, dir);
  if (expected_descent >= 0 || !math::isfinite(expected_descent)) {
    throw make_runtime_error("Invalid descent direction: Dot[grad, dir]={}", expected_descent);
  }

  Index iter = 0;
  Variable x;
  Real f = math::inf<Real>;
  for (; iter < max_iter_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    if (prob.HasProximator()) {
      x = prob.EvalProximator(x, alpha);
      expected_descent = math::dot(grad, x - x0);
    }
    f = prob.EvalEnergy(x);
    if (examine_arjimo_condition(f, f0, alpha * expected_descent, required_descent_rate_)
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

void Linesearch_Backtracking::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT(options, Real, initial_step_length);
  AX_SYNC_OPT(options, Real, step_shrink_rate);
  AX_SYNC_OPT(options, Real, required_descent_rate);
  LinesearchBase::SetOptions(options);
}

utils::Options Linesearch_Backtracking::GetOptions() const {
  utils::Options opt = LinesearchBase::GetOptions();
  opt["initial_step_length"] = initial_step_length_;
  opt["step_shrink_rate"] = step_shrink_rate_;
  opt["required_descent_rate"] = required_descent_rate_;
  return opt;
}

LineSearchKind Linesearch_Backtracking::GetKind() const { return LineSearchKind::kBacktracking; }
}  // namespace ax::optim
