// #define SPDLOG_ACTIVE_LEVEL 0
#include "ax/optim/linesearch/wolfe.hpp"

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

static Real fit_quadratic_and_solve(Real f_a, Real df_a, Real f_b, Real a, Real b) {
  // See Page 58, Numerical Optimization, Nocedal and Wright. Eq. 3.57
  // f(x) = B (x-a)^2 + C (x-a) + D
  // f(a) = D.
  // f'(a) = C.
  // f(b) = B (b-a)^2 + C (b-a) + D => B = (f(b) - f(a) - C (b-a)) / (b-a)^2
  // optimal = a - C / (2 B)
  Real b_a = b - a;
  Real C = df_a, D = f_a;
  Real B = (f_b - f_a - C * b_a) / (b_a * b_a);
  return a - C / (2 * B);
}

static Real fit_cubic_and_solve(Real f_a, Real df_a, Real f_b, Real f_c, Real a, Real b, Real c) {
  // See Page 58, Numerical Optimization, Nocedal and Wright. Eq. 3.58
  // f(x) = A (x-a)^3 + B (x-a)^2 + C (x-a) + D
  Real C = df_a;
  Real db = b - a, dc = c - a;
  Real denom = math::square(db * dc) * (db - dc);
  math::RealMatrix2 d1;
  d1(0, 0) = dc * dc;
  d1(0, 1) = -db * db;
  d1(1, 0) = -dc * dc * dc;
  d1(1, 1) = db * db * db;
  math::RealVector2 d2;
  d2[0] = f_b - f_a - C * db;
  d2[1] = f_c - f_a - C * dc;
  math::RealVector2 x = d1 * d2;
  Real A = x[0] / denom, B = x[1] / denom;
  Real radical = B * B - 3 * A * C;
  Real xmin = a + (-B + std::sqrt(radical)) / (3 * A);
  return xmin;
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

  Index iter = 0;
  Variable x;

  Real alpha = initial_step_size_;
  Real alpha_last = 0.;
  Real f = math::nan<>;
  Real f_last = f0;
  Real g_dot_dir = math::nan<>;
  Real g_dot_dir_last = expected_descent;
  bool success = false;

  // Implements algorithm 3.6
  auto zoom = [&](Real lo, Real hi, Real f_at_lo, Real g_at_lo, Real f_at_hi) {
    Real a, f_at_a;
    Real a_last = math::nan<>, f_last = math::nan<>;
    success = false;
    Real epsilon = 1e-6;
    for (int i = 0; i < 100; ++i) {
      if (verbose_) {
        AX_INFO("Zooming: i={:3} lo={:12.6e} hi={:12.6e}", i, lo, hi);
      }

      // follow the code in scipy/optimize/_linesearch.py:573
      auto [mi, ma] = std::minmax(lo, hi);
      if (i > 0) {
        // Try to solve cubic.
        a = fit_cubic_and_solve(f_at_lo, g_at_lo, f_at_hi, f_last, lo, hi, a_last);
      }
      if ((i == 0) || a < mi + epsilon || a > ma - epsilon || !math::isfinite(a)) {
        a = fit_quadratic_and_solve(f_at_lo, g_at_lo, f_at_hi, lo, hi);
        AX_DEBUG("Fit result {:12.6e}", a);
        if (!math::isfinite(a) || a < mi + epsilon || a > ma - epsilon) {
          a = step_shrink_rate_ * hi + (1 - step_shrink_rate_) * lo;
        }
      }
      AX_DEBUG("i={:3} lo={:12.6e} hi={:12.6e} a={:12.6e}", i, lo, hi, a);
      // Line 2 Check the termination condition:
      f_at_a = prob.EvalEnergy(x0 + a * dir);                           // Line 2
      if (f_at_a >= f0 + required_descent_rate_ * a * expected_descent  // Line 3
          || f_at_a >= f_at_lo) {                                       // Line 3
        AX_DEBUG("f_at_a={:12.6e} f0={:12.6e} expected_descent={:12.6e}", f_at_a, f0,
                 expected_descent);
        a_last = hi;
        f_last = f_at_hi;
        hi = a;            // step size is still too large
        f_at_hi = f_at_a;  // Line 4
      } else {             // Line 5
        Real grad_at_a = math::dot(prob.EvalGrad(x0 + a * dir), dir);                // Line 6
        if (math::abs(grad_at_a) <= -required_curvature_rate_ * expected_descent) {  // Line 7
          success = true;
          break;  // Satisfies wolfe.
        }
        if (grad_at_a * (hi - lo) >= 0) {  // Line 8
          a_last = hi;
          f_last = f_at_hi;
          hi = lo;  // Line 9
          f_at_hi = f_at_lo;
        }
        a_last = lo;
        f_last = f_at_lo;
        lo = a;  // Line 10
        f_at_lo = f_at_a;
        g_at_lo = grad_at_a;
      }
    }
    if (!success) {
      AX_CRITICAL("Zoom failed");
    }
    return std::make_pair(a, f_at_a);
  };

  // Implements algorithm 3.5
  for (; iter < max_iter_ && alpha < max_step_size_; ++iter) {
    x.noalias() = x0 + alpha * dir;
    f = prob.EvalEnergy(x);
    const bool is_too_large = f > f0 + required_descent_rate_ * alpha * expected_descent;
    const bool is_not_descent = f > f_last && iter > 1;
    if (verbose_) {
      AX_INFO("Outer {:3}: alpha_last={:12.6e} alpha={:12.6e} f={:12.6e} f_last={:12.6e}", iter,
              alpha_last, alpha, f, f_last);
    }

    if (is_too_large || is_not_descent) {  // Line 5
      AX_DEBUG("Too large step, find a proper alpha between alpha_last and alpha!");
      std::tie(alpha, f) = zoom(alpha_last, alpha, f_last, g_dot_dir_last, f);  // Line 6
      break;
    }

    Gradient g = prob.EvalGrad(x);  // Line 7
    g_dot_dir = math::dot(g, dir);
    AX_DEBUG("Alpha={:12.6e} f={:12.6e} g_dot_dir={:12.6e} iter={}", alpha, f, g_dot_dir, iter);
    if (math::abs(g_dot_dir) <= -required_curvature_rate_ * expected_descent) {  // Line 8
      AX_DEBUG("Satisfies strong wolfe. break directly");
      success = true;
      break;  // Line 9
    }

    if (g_dot_dir >= 0.) {  // Line 10
      AX_DEBUG("Too small, zooming and break");
      std::tie(alpha, f) = zoom(alpha, alpha_last, f, g_dot_dir, f_last);  // Line 11
      break;
    }

    g_dot_dir_last = g_dot_dir;
    alpha_last = alpha;
    f_last = f;
    alpha *= step_expand_rate_;
  }

  OptResult opt;
  opt.x_opt_ = x0 + alpha * dir;  // cannot be x directly, because we may zoom alpha then break.
  opt.f_opt_ = f;
  opt.n_iter_ = iter;
  opt.converged_ = success;
  opt.step_length_ = alpha;
  return opt;
}

void Linesearch_Wofle::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT(options, Real, step_shrink_rate);
  AX_SYNC_OPT(options, Real, step_expand_rate);
  AX_SYNC_OPT(options, Real, required_descent_rate);
  AX_SYNC_OPT(options, Real, required_curvature_rate);
  LinesearchBase::SetOptions(options);
}

utils::Options Linesearch_Wofle::GetOptions() const {
  utils::Options opt = LinesearchBase::GetOptions();
  opt["step_shrink_rate"] = step_shrink_rate_;
  opt["step_expand_rate"] = step_expand_rate_;
  opt["required_descent_rate"] = required_descent_rate_;
  opt["required_curvature_rate"] = required_curvature_rate_;
  return opt;
}

LineSearchKind Linesearch_Wofle::GetKind() const {
  return LineSearchKind::kWolfe;
}

}  // namespace ax::optim
