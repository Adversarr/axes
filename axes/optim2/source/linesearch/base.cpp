#include "ax/optim2/linesearch/base.hpp"

#include "ax/math/buffer_blas.hpp"

namespace ax::optim2 {

#define FIX_PARAM_IMPL(name) param.name = param.name.value_or(name)

Real LineSearchBase::GetCurrentStep() const {
  return current_step_size_;
}

void LineSearchBase::StepTo(const LineSearchParam& param, Real step_size, bool update_gradient, size_t iter) {
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

  if (grad_dot_dir_x0_ >= 0) {
    AX_THROW_RUNTIME_ERROR(
        "Invalid search direction, not a descent direction: grad dot dir={:12.6e}",
        grad_dot_dir_x0_);
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
  AX_EXPECTS(armijo < 0);
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
  return (lo + hi) * shrink;
}

void LineSearchBase::FixParameter(LineSearchParam& param) const noexcept {
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
}

}  // namespace ax::optim2