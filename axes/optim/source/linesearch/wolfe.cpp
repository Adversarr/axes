#include "ax/optim/linesearch/wolfe.hpp"

#include "ax/core/excepts.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/opt.hpp"
namespace ax::optim {
OptResult Linesearch_Wofle::Optimize(OptProblem const& prob, Variable const& x0,
                                     Gradient const& grad, Variable const& dir) const {
  // SECT: Check Inputs
  AX_THROW_IF_LT(initial_step_size_, 0, "Initial step length must be positive");
  AX_THROW_IF_FALSE(0 < step_shrink_rate_ && step_shrink_rate_ < 1,
                    "Step shrink rate must be in (0, 1)");
  AX_THROW_IF_FALSE(0 < required_descent_rate_ && required_descent_rate_ < 1,
                    "Required descent rate must be in (0, 1)");

  AX_THROW_IF_FALSE(prob.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(prob.HasEnergy(), "Energy function not set");

  Real alpha = initial_step_size_;
  Real const f0 = prob.EvalEnergy(x0);
  Real const expected_descent = math::dot(grad, dir);

  AX_THROW_IF_LE(alpha, min_step_size_, "Initial step size is too small ({} < {})", alpha,
                 min_step_size_);
  AX_THROW_IF_FALSE(math::isfinite(f0),
                    "Invalid x0 in Line Search, Energy returns infinite number.");
  AX_THROW_IF_FALSE(expected_descent >= 0 || !math::isfinite(expected_descent),
                    "Invalid descent direction: df0={}", expected_descent);

  Index iter = 0;
  Variable x;
  Real f = f0;
  for (; iter < max_iter_ && alpha > min_step_size_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    auto g = prob.EvalGrad(x);
    f = prob.EvalEnergy(x);
    if (math::isfinite(f)) {
      bool arjimo
          = examine_arjimo_condition(f, f0, alpha * expected_descent, required_descent_rate_);
      bool curvature
          = examine_curvature_condition(dir, g, expected_descent, required_curvature_rate_);
      bool strong_wofle
          = !strong_wolfe_
            || examine_strong_wolfe_condition(dir, g, expected_descent, required_curvature_rate_);

      if (arjimo && curvature && strong_wofle) {
        break;
      }
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

void Linesearch_Wofle::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT(options, Real, step_shrink_rate);
  AX_SYNC_OPT(options, Real, required_descent_rate);
  AX_SYNC_OPT(options, Real, required_curvature_rate);
  AX_SYNC_OPT(options, bool, strong_wolfe);
  LinesearchBase::SetOptions(options);
}

utils::Options Linesearch_Wofle::GetOptions() const {
  utils::Options opt = LinesearchBase::GetOptions();
  opt["step_shrink_rate"] = step_shrink_rate_;
  opt["required_descent_rate"] = required_descent_rate_;
  opt["strong_wolfe"] = strong_wolfe_;
  return opt;
}

LineSearchKind Linesearch_Wofle::GetKind() const {
  return LineSearchKind::kWolfe;
}
}  // namespace ax::optim
