#include "ax/optim/linesearch/exact.hpp"

#include <iostream>

#include "ax/core/logging.hpp"

namespace ax::optim {

LineSearchKind Linesearch_Exact::GetKind() const {
  return LineSearchKind::kExact;
}

OptResult Linesearch_Exact::Optimize(OptProblem const& prob, Variable const& x0,
                                     Gradient const& grad, Variable const& dir) const {
  AX_THROW_IF_LT(initial_step_size_, 0, "Initial step length must be positive");
  AX_THROW_IF_FALSE(prob.HasEnergy(), "Energy function not set");

  Real const f0 = prob.EvalEnergy(x0);
  AX_THROW_IF_FALSE(math::isfinite(f0),
                    "Invalid x0 in Line Search, Energy returns infinite number.");

  Real left = 0.;
  Real right = initial_step_size_;
  Index iter = 0;

  static constexpr Real kGoldenRatio = 0.618033988749895;

  // compute approximately how many step we need to reach accarcy
  Real const max_required_iter
      = std::log(required_accuracy_ / initial_step_size_) / std::log(kGoldenRatio);
  if (static_cast<Real>(max_iter_) < max_required_iter) {
    AX_WARN("Max iteration is too small to reach required accuracy, consider increase max_iter");
  }

  Real optimal_step_size = 0;
  Real optimal_f = f0;
  auto update_optimal = [&optimal_step_size, &optimal_f](Real step, Real f) {
    if (f < optimal_f && math::isfinite(f)) {
      optimal_step_size = step;
      optimal_f = f;
    }
  };

  for (; iter < max_iter_; ++iter) {
    Real mid_left = left + (1 - kGoldenRatio) * (right - left);
    Real f_mid_left = prob.EvalEnergy(x0 + mid_left * dir);
    Real mid_right = left + kGoldenRatio * (right - left);
    Real f_mid_right = prob.EvalEnergy(x0 + mid_right * dir);

    if (verbose_) {
      AX_INFO(
          "ExactLS iter {:3}: interval {:12.6e}, {:12.6e} mid {:12.6e} ({:12.6e}), "
          "{:12.6e}({:12.6e}), cur_optimal {:12.6e}({:12.6e})",
          iter, left, right, mid_left, f_mid_left, mid_right, f_mid_right, optimal_step_size,
          optimal_f);
    }

    update_optimal(mid_left, f_mid_left);
    update_optimal(mid_right, f_mid_right);

    bool is_mid_left_finite = math::isfinite(f_mid_left);
    bool is_mid_right_finite = math::isfinite(f_mid_right);
    bool is_left_less = false;  // check for NaN
    if (is_mid_left_finite && is_mid_right_finite) {
      is_left_less = f_mid_left < f_mid_right;  // No NaN
    } else if (is_mid_left_finite) {
      is_left_less = true;  // Mid right is NaN, use mid left
    } else if (is_mid_right_finite) {
      is_left_less = false;  // Mid left is NaN, use mid right
    } else {
      is_left_less = true;  // Both are NaN, will force to make step size smaller.
    }

    if (is_left_less) {
      // Move right to mid_right
      right = mid_right;
    } else {
      // Move left to mid_left
      left = mid_left;
    }

    if ((left + right) * 0.5 < min_step_size_) {
      break;
    }
    if (right - left < required_accuracy_) {
      break;
    }
  }

  Real step_size = (left + right) * 0.5;
  Variable final_x = x0 + step_size * dir;
  Real final_f = prob.EvalEnergy(final_x);
  update_optimal(step_size, final_f);

  if (math::isfinite(optimal_f) && optimal_f < f0) {
    return OptResult::ConvergedLinesearch(x0 + optimal_step_size * dir, final_f, iter, step_size);
  } else {
    return OptResult::NotConverged("Exact line search failed to find a descent direction");
  }
}

utils::Options Linesearch_Exact::GetOptions() const {
  utils::Options options = LinesearchBase::GetOptions();
  options.emplace("initial_step_size", initial_step_size_);
  options.emplace("required_accuracy", required_accuracy_);
  return options;
}

void Linesearch_Exact::SetOptions(utils::Options const& option) {
  AX_SYNC_OPT(option, Real, initial_step_size);
  AX_SYNC_OPT(option, Real, required_accuracy);
  LinesearchBase::SetOptions(option);
}

}  // namespace ax::optim