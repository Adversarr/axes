#include "ax/optim2/linesearch/wolfe.hpp"

#include "ax/math/buffer_blas.hpp"

namespace ax::optim2 {

static void wolfe_warning() {
  static std::once_flag flag;
  std::call_once(flag, []() {
    AX_WARN(
        "Prefer Backtracking in most cases. Strong Wolfe linesearch does not always guarantee a "
        "better performance.");
  });
}

LineSearchResult LineSearch_Wolfe::Optimize(LineSearchParam param) {
  LineSearchBase::BeginSearch(param);
  LineSearchResult result;
  result.f_opt_ = CurrentEnergy();
  result.n_iter_ = 0;

  wolfe_warning();

  size_t& iter = result.n_iter_;
  Real original_energy = OriginalEnergy();
  Real original_grad_dot_dir = OriginalGradientDotDirection();
  Real current_step = GetCurrentStep();
  Real last_step = 0, last_energy = original_energy, last_grad_dot_dir = original_grad_dot_dir;

  const Real required_descent_rate = param.armijo_.value();
  const Real armijo_threshold = required_descent_rate * original_grad_dot_dir;
  const Real curvature_threshold = math::abs(param.curvature_.value() * original_grad_dot_dir);

  AX_INFO("Thresholds: armijo={} curvature={}", armijo_threshold, curvature_threshold);

  Real current_g_dot_dir = math::nan<>;
  Real zoom_left = math::nan<>, zoom_right = math::nan<>;
  Real left_energy = math::nan<>, right_energy = math::nan<>;
  Real left_grad_dot_dir = math::nan<>;
  bool need_zoom = true;

  for (size_t max_iter = param.max_iter_.value(); iter < max_iter; ++iter) {
    // Go to the next step: x <- x0 + step * dir
    StepTo(param, current_step, false, iter);
    Real f = CurrentEnergy();
    const bool is_too_large = f > original_energy + current_step * armijo_threshold;
    const bool is_not_descent = f >= original_energy && iter > 0;

    if (is_not_descent || is_too_large) {
      AX_DEBUG("Too large step, find a proper alpha between alpha_last and alpha!");
      zoom_left = last_step;  // alpha_i-1
      left_energy = last_energy;
      left_grad_dot_dir = last_grad_dot_dir;
      zoom_right = current_step;  // alpha_i
      right_energy = f;
      break;
    }

    UpdateGradient();  // since StepTo does not update it.
    current_g_dot_dir = math::buffer_blas::dot(CurrentGradient(), param.search_direction_);

    // Strong wolfe condition.
    if (math::abs(current_g_dot_dir) <= curvature_threshold) {
      AX_TRACE("Satisfies strong wolfe. break directly");
      result.step_length_ = current_step;
      result.converged_strong_wolfe_ = true;
      result.converged_ = true;
      need_zoom = false;  // strong wolfe reached, no need to zoom.
      break;
    }

    if (current_g_dot_dir >= 0) {
      zoom_left = current_step;
      left_energy = f;
      left_grad_dot_dir = current_g_dot_dir;
      zoom_right = last_step;
      right_energy = last_energy;
      break;
    }

    // Enlarge current step size.
    Real next_step = -1;
    Real next_step_min = current_step * 1.01;
    Real next_step_max = param.max_step_expand_factor_.value() * current_step;

    if (param.interpolation_kind_ == LinesearchInterpolationKind::Quadratic) {
      next_step = SolveOptimalStepSizeQuadratic(param, last_energy, last_grad_dot_dir, f, last_step,
                                                current_step);
      if (!math::isfinite(next_step)) {
        next_step = SolveOptimalStepSizeNone(param, next_step_min, next_step_max);
      } else {
        next_step = std::clamp(next_step, next_step_min, next_step_max);
      }
    } else if (param.interpolation_kind_ == LinesearchInterpolationKind::Cubic) {
      AX_NOT_IMPLEMENTED();
    } else {
      next_step = SolveOptimalStepSizeNone(param, last_step, current_step);
    }
    AX_EXPECTS(next_step >= next_step_min && next_step <= next_step_max);

    last_step = current_step;
    last_energy = f;
    last_grad_dot_dir = current_g_dot_dir;
    current_step = next_step;
  }

  // zoom stage.
  for (; iter < max_iter_ && need_zoom; ++iter) {
    // find next step.
    Real lo = zoom_left, hi = zoom_right;
    Real f_at_lo = left_energy, f_at_hi = right_energy;
    Real g_at_lo = left_grad_dot_dir;

    constexpr Real delta2 = 0.1;  // quadratic interpolation, see scipy wolfe zoom.
    auto [mi, ma] = std::minmax(lo, hi);
    Real epsilon = (ma - mi) * delta2;
    Real next_step = -1;
    if (param.interpolation_kind_ == LinesearchInterpolationKind::Quadratic) {
      next_step = SolveOptimalStepSizeQuadratic(param, f_at_lo, g_at_lo, f_at_hi, lo, hi);
      if (!math::isfinite(next_step) || next_step < mi + epsilon || next_step > ma - epsilon) {
        next_step = SolveOptimalStepSizeNone(param, lo, hi);
      }
    } else if (param.interpolation_kind_ == LinesearchInterpolationKind::Cubic) {
      AX_NOT_IMPLEMENTED();
    } else {
      next_step = SolveOptimalStepSizeNone(param, lo, hi);
    }

    // evaluate object function.
    StepTo(param, next_step, false, iter);
    Real f_next = CurrentEnergy();
    if (!TestCurrentArmojo(param) || f_next >= f_at_lo) {
      // could be lower. continue.
      zoom_right = next_step;
      right_energy = f_next;
    } else {
      UpdateGradient();
      Real g_next = math::buffer_blas::dot(CurrentGradient(), param.search_direction_);
      if (math::abs(g_next) <= curvature_threshold) {
        result.step_length_ = next_step;
        result.converged_strong_wolfe_ = true;
        result.converged_ = true;
        break;
      } else if (g_next * (hi - lo) >= 0) {
        zoom_right = lo;
        right_energy = f_at_lo;
      }

      zoom_left = next_step;
      left_energy = f_next;
      left_grad_dot_dir = g_next;
    }
  }

  EndSearch(result);
  return result;
}

LineSearchKind LineSearch_Wolfe::GetKind() const {
  return LineSearchKind::Wolfe;
}

}  // namespace ax::optim2