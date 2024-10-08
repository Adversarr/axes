#include "ax/optim2/linesearch/base.hpp"

#include "ax/math/buffer_blas.hpp"

namespace ax::optim2 {

#define FIX_PARAM_IMPL(name) param.name = param.name.value_or(name)

Real LineSearchBase::GetCurrentStep() const {
  return current_step_size_;
}

void LineSearchBase::StepTo(const LineSearchParam& param, Real step_size, bool update_gradient,
                            size_t iter) {
  current_step_size_ = step_size;

  // x <- x0 + step * dir
  problem_->StepBack();  // go back to x0
  problem_->StepTemporary(param.search_direction_, step_size, 1.0);
  problem_->OnStep(true, iter);

  problem_->UpdateEnergy();
  if (update_gradient) {
    problem_->UpdateGradient();
  }
}

Real LineSearchBase::OriginalEnergy() const {
  return problem_->GetBackupEnergy();
}

Real LineSearchBase::OriginalGradientDotDirection() const {
  return grad_dot_dir_x0_;
}

ConstRealBufferView LineSearchBase::OriginalGradient() const {
  return problem_->GetBackupGradient();
}

ConstRealBufferView LineSearchBase::OriginalVariables() const {
  return problem_->GetBackupVariables();
}

Real LineSearchBase::CurrentEnergy() const {
  return problem_->GetEnergy();
}

void LineSearchBase::BeginSearch(LineSearchParam& param) {
  FixParameter(param);
  current_step_size_ = *param.initial_step_size_;
  auto grad = problem_->GetGradient();

  grad_dot_dir_x0_ = math::buffer_blas::dot(grad, param.search_direction_);

  if (grad_dot_dir_x0_ >= 0 || !math::isfinite(grad_dot_dir_x0_)) {
    AX_THROW_RUNTIME_ERROR(
        "Invalid search direction, not a descent direction: grad dot dir={:12.6e} |g|={:12.6e}, "
        "|d|={:12.6e}",
        grad_dot_dir_x0_, math::buffer_blas::norm(grad),
        math::buffer_blas::norm(param.search_direction_));
  }
}

void LineSearchBase::EndSearch(LineSearchResult& result) {
  result.f_opt_ = CurrentEnergy();
  result.step_length_ = GetCurrentStep();
}

static bool arjimo_condition(Real f0, Real f_step, Real step_size, Real armijo_threshold) {
  return f_step <= f0 + step_size * armijo_threshold;
}

static bool curvature_condition(Real grad_dot_dir, Real grad_dot_dir0, Real curvature) {
  // grad_dot_dir0 is negative.
  return grad_dot_dir >= grad_dot_dir0 * curvature;
}

static bool strong_wolfe_condition(Real grad_norm, Real grad_norm0, Real strong_wolfe) {
  return grad_norm <= grad_norm0 * strong_wolfe;
}

bool LineSearchBase::TestCurrentArmojo(const LineSearchParam& param) const {
  Real f_step = CurrentEnergy();
  Real f0 = OriginalEnergy();
  Real armijo = param.armijo_.value() * OriginalGradientDotDirection();
  return arjimo_condition(f0, f_step, current_step_size_, armijo);
}

bool LineSearchBase::TestCurrentCurvature(const LineSearchParam& param) const {
  Real grad_dot_dir = math::buffer_blas::dot(problem_->GetGradient(), param.search_direction_);
  Real grad_dot_dir0 = OriginalGradientDotDirection();
  Real curvature = param.curvature_.value();
  return curvature_condition(grad_dot_dir, grad_dot_dir0, curvature);
}

bool LineSearchBase::TestCurrentStrongWolfe(const LineSearchParam& param) const {
  Real grad_norm = math::buffer_blas::norm(problem_->GetGradient());
  Real grad_norm0 = math::buffer_blas::norm(OriginalGradient());
  Real strong_wolfe = param.strong_wolfe_.value();
  return strong_wolfe_condition(grad_norm, grad_norm0, strong_wolfe);
}

void LineSearchBase::SetProblem(shared_not_null<ProblemBase> problem) {
  problem_ = problem;
}

Real LineSearchBase::SolveOptimalStepSizeQuadratic(const LineSearchParam& /* param */, Real f_lo,
                                                   Real g_lo, Real f_hi, Real lo, Real hi) const {
  // See Page 58, Numerical Optimization, Nocedal and Wright. Eq. 3.57
  // f(x) = B (x-a)^2 + C (x-a) + D
  // f(a) = D.
  // f'(a) = C.
  // f(b) = B (b-a)^2 + C (b-a) + D => B = (f(b) - f(a) - C (b-a)) / (b-a)^2
  // optimal = a - C / (2 B)

  const Real b = hi, a = lo, df_a = g_lo, f_a = f_lo, f_b = f_hi;
  Real b_a = b - a;
  Real c = df_a /* , d = f_a */;
  Real bb = (f_b - f_a - c * b_a) / (b_a * b_a);
  return a - c / (2 * bb);
}

Real LineSearchBase::SolveOptimalStepSizeNone(const LineSearchParam& param, Real lo,
                                              Real hi) const {
  Real shrink = param.step_shrink_factor_.value();
  return shrink * hi + (1 - shrink) * lo;
}

void LineSearchBase::FixParameter(LineSearchParam& param) const {
  FIX_PARAM_IMPL(min_step_size_);
  FIX_PARAM_IMPL(max_step_size_);
  FIX_PARAM_IMPL(initial_step_size_);
  FIX_PARAM_IMPL(armijo_);
  FIX_PARAM_IMPL(curvature_);
  FIX_PARAM_IMPL(strong_wolfe_);
  FIX_PARAM_IMPL(step_shrink_factor_);
  FIX_PARAM_IMPL(min_step_shrink_factor_);
  FIX_PARAM_IMPL(max_step_shrink_factor_);
  FIX_PARAM_IMPL(step_expand_factor_);
  FIX_PARAM_IMPL(max_step_expand_factor_);
  FIX_PARAM_IMPL(interpolation_kind_);
  FIX_PARAM_IMPL(max_iter_);

  if (param.max_step_size_ <= param.min_step_size_) {
    AX_THROW_INVALID_ARGUMENT("max_step_size_ must be greater than min_step_size_, got {} and {}",
                              param.max_step_size_.value(), param.min_step_size_.value());
  }
  if (param.armijo_ <= 0 || param.armijo_ >= 1) {
    AX_THROW_INVALID_ARGUMENT("armijo_ must be in the range (0, 1), got {}", param.armijo_.value());
  }
  if (param.curvature_ <= 0 || param.curvature_ >= 1) {
    AX_THROW_INVALID_ARGUMENT("curvature_ must be in the range (0, 1), got {}",
                              param.curvature_.value());
  }
  if (param.step_shrink_factor_ <= 0 || param.step_shrink_factor_ >= 1) {
    AX_THROW_INVALID_ARGUMENT("step_shrink_factor_ must be in the range (0, 1), got {}",
                              param.step_shrink_factor_.value());
  }
  if (param.min_step_shrink_factor_ <= 0 || param.min_step_shrink_factor_ >= 1) {
    AX_THROW_INVALID_ARGUMENT("min_step_shrink_factor_ must be in the range (0, 1), got {}",
                              param.min_step_shrink_factor_.value());
  }
  if (param.max_step_shrink_factor_ <= 0 || param.max_step_shrink_factor_ >= 1) {
    AX_THROW_INVALID_ARGUMENT("max_step_shrink_factor_ must be in the range (0, 1), got {}",
                              param.max_step_shrink_factor_.value());
  }
  if (param.step_expand_factor_ <= 1) {
    AX_THROW_INVALID_ARGUMENT("step_expand_factor_ must be greater than 1, got {}",
                              param.step_expand_factor_.value());
  }
  if (param.max_step_expand_factor_ <= param.step_expand_factor_) {
    AX_THROW_INVALID_ARGUMENT(
        "max_step_expand_factor_ must be greater than step_expand_factor_, "
        "got {} and {}",
        param.max_step_expand_factor_.value(), param.step_expand_factor_.value());
  }
}

void LineSearchBase::UpdateGradient() {
  problem_->UpdateGradient();
}

ConstRealBufferView LineSearchBase::CurrentGradient() const {
  return problem_->GetGradient();
}

utils::Options LineSearchBase::GetOptions() const {
  auto opt = utils::Tunable::GetOptions();
  opt["min_step_size"] = min_step_size_;
  opt["max_step_size"] = max_step_size_;
  opt["initial_step_size"] = initial_step_size_;
  opt["armijo"] = armijo_;
  opt["curvature"] = curvature_;
  opt["strong_wolfe"] = strong_wolfe_;
  opt["step_shrink_factor"] = step_shrink_factor_;
  opt["min_step_shrink_factor"] = min_step_shrink_factor_;
  opt["max_step_shrink_factor"] = max_step_shrink_factor_;
  opt["step_expand_factor"] = step_expand_factor_;
  opt["max_step_expand_factor"] = max_step_expand_factor_;
  opt["interpolation_kind"] = utils::reflect_name(interpolation_kind_).value();
  opt["max_iter"] = max_iter_;
  return opt;
}

void LineSearchBase::SetOptions(const utils::Options& opt) {
  AX_SYNC_OPT(opt, Real, min_step_size);
  AX_SYNC_OPT(opt, Real, max_step_size);
  AX_SYNC_OPT(opt, Real, initial_step_size);
  AX_SYNC_OPT(opt, Real, armijo);
  AX_SYNC_OPT(opt, Real, curvature);
  AX_SYNC_OPT(opt, Real, strong_wolfe);
  AX_SYNC_OPT(opt, Real, step_shrink_factor);
  AX_SYNC_OPT(opt, Real, min_step_shrink_factor);
  AX_SYNC_OPT(opt, Real, max_step_shrink_factor);
  AX_SYNC_OPT(opt, Real, step_expand_factor);
  AX_SYNC_OPT(opt, Real, max_step_expand_factor);
  AX_SYNC_OPT_ENUM(opt, LinesearchInterpolationKind, interpolation_kind_, interpolation_kind);
  AX_SYNC_OPT(opt, size_t, max_iter);

  utils::Tunable::SetOptions(opt);
}

}  // namespace ax::optim2