#include "ax/optim2/linesearch/backtrack.hpp"

namespace ax::optim2 {

LineSearch_Backtracking::LineSearch_Backtracking() = default;

LineSearchResult LineSearch_Backtracking::Optimize(LineSearchParam param) {
  LineSearchBase::BeginSearch(param);
  LineSearchResult result;
  result.f_opt_ = CurrentEnergy();
  result.n_iter_ = 0;

  size_t& iter = result.n_iter_;

  Real original_energy = OriginalEnergy();
  Real original_grad_dot_dir = OriginalGradientDotDirection();
  Real current_step = GetCurrentStep();
  for (size_t max_iter = param.max_iter_.value(); iter < max_iter; ++iter) {
    // Go to the next step: x <- x0 + step * dir
    StepTo(param, current_step, false);
    Real value = CurrentEnergy();
    // do the armijo test.
    result.converged_ = TestCurrentArmojo(param);
    if (result.converged_) {
      break;
    }

    // Determine next step size.
    Real next_step = -1;
    Real next_step_min = param.min_step_shrink_factor_.value() * current_step;
    Real next_step_max = param.max_step_shrink_factor_.value() * current_step;

    if (param.interpolation_kind_ == LinesearchInterpolationKind::Quadratic) {
      next_step = SolveOptimalStepSizeQuadratic(param, original_energy, original_grad_dot_dir,
                                                value, 0, current_step);
      if (!math::isfinite(next_step)) {
        next_step = SolveOptimalStepSizeNone(param, next_step_min, next_step_max);
      } else {
        next_step = std::clamp(next_step, next_step_min, next_step_max);
      }
    } else if (param.interpolation_kind_ == LinesearchInterpolationKind::Cubic) {
      AX_NOT_IMPLEMENTED();
    } else {
      next_step = SolveOptimalStepSizeNone(param, 0, current_step);
    }

    AX_EXPECTS(next_step_min <= next_step && next_step <= next_step_max);
    current_step = next_step;
  }

  EndSearch(result);
  return result;
}

}  // namespace ax::optim2