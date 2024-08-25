#include "ax/optim/linesearch/wolfe.hpp"

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

bool Linesearch_Wofle::SatisfiesTerminationCondition(const Variable& dir, const Real& step_length,
                                                     const Real& f0, const Real& expected_descent,
                                                     const Real& f, const Gradient& g) const {
  if (math::isfinite(f)) {
    bool arjimo
        = examine_arjimo_condition(f, f0, step_length * expected_descent, required_descent_rate_);
    bool curvature
        = examine_curvature_condition(dir, g, expected_descent, required_curvature_rate_);
    bool strong_wofle
        = !strong_wolfe_
          || examine_strong_wolfe_condition(dir, g, expected_descent, required_curvature_rate_);

    if (arjimo && curvature && strong_wofle) {
      return true;
    }
  }
  return false;
}

Real fit_quadratic_and_solve(Real f_at_0, Real df_at_0, Real f_at_a, Real a) {
  // See Page 58, Numerical Optimization, Nocedal and Wright. Eq. 3.57
  return -(df_at_0 * a * a) / (2 * (f_at_a - f_at_0 - df_at_0 * a));
}

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

  Real const f0 = prob.EvalEnergy(x0);
  Real const expected_descent = math::dot(grad, dir);  // Also the directional derivative f'(0)

  AX_THROW_IF_FALSE(math::isfinite(f0),
                    "Invalid x0 in Line Search, Energy returns infinite number.");
  AX_THROW_IF_TRUE(expected_descent >= 0 || !math::isfinite(expected_descent),
                   "Invalid descent direction: expected_descent={:12.6e}", expected_descent);

  auto select_alpha = [](Real l, Real h) {
    return 0.5 * (l + h);
  };

  Real alpha_last = 0.;
  Real alpha = select_alpha(alpha_last, initial_step_size_);  // Line 0.

  Index iter = 0;
  Variable x;
  Real f = f0;
  Real f_last = f0;
  Real g_dot_dir = expected_descent;
  Real g_dot_dir_last = expected_descent;

  // Implements algorithm 3.6
  auto zoom = [&](Real lo, Real hi, Real f_at_lo, Real g_at_lo, Real f_at_hi) -> Real {
    Real a;
    for (int i = 0; i < 100; ++i) {
      // Line 1 Interpolate:
      // a = fit_quadratic_and_solve(f_at_lo, g_at_lo, f_at_hi, hi - lo) + lo;
      a = 0.5 * (hi + lo);
      AX_INFO("i={} a={}", i, a);
      // Line 2 Check the termination condition:
      Real f_at_a = prob.EvalEnergy(x0 + a * dir);                      // Line 2
      if (f_at_a >= f0 + required_descent_rate_ * a * expected_descent  // Line 3
          || f_at_a >= f_at_lo) {                                       // Line 3
        hi = a;
        f_at_hi = f_at_a;                                                            // Line 4
      } else {                                                                       // Line 5
        Real grad_at_a = math::dot(prob.EvalGrad(x0 + a * dir), dir);                // Line 6
        if (math::abs(grad_at_a) <= -required_curvature_rate_ * expected_descent) {  // Line 7
          break;
        }
        if (grad_at_a * (hi - lo) >= 0) {  // Line 8
          hi = lo;                         // Line 9
          f_at_hi = f_at_lo;
        }
        lo = a;  // Line 10
        f_at_lo = f_at_a;
        g_at_lo = grad_at_a;
      }
    }
    return a;
  };

  // Implements algorithm 3.5
  for (; iter < max_iter_ && alpha > min_step_size_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    f = prob.EvalEnergy(x);
    const bool is_converged  // Test the arjimo condition.
        = math::isfinite(f)
          && examine_arjimo_condition(f, f0, alpha * expected_descent, required_descent_rate_);
    const bool is_not_descent = f > f_last && iter > 1;
    if (is_converged || is_not_descent) {
      alpha = zoom(alpha_last, alpha, f_last, g_dot_dir_last, f);  // Line 6
      break;
    }

    Gradient g = prob.EvalGrad(x);  // Line 7
    g_dot_dir_last = g_dot_dir;
    g_dot_dir = math::dot(g, dir);
    if (math::abs(g_dot_dir) <= -required_curvature_rate_ * expected_descent) {  // Line 8
      break;                                                                     // Line 9
    }

    if (g_dot_dir >= 0) {                                     // Line 10
      alpha = zoom(alpha, alpha_last, f, g_dot_dir, f_last);  // Line 11
      break;
    }

    alpha_last = alpha;
    f_last = f;
    alpha = select_alpha(alpha_last, initial_step_size_);  // Line 12
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
