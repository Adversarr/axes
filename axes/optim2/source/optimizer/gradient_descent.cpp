#include "ax/optim2/optimizer/gradient_descent.hpp"

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/linesearch/backtrack.hpp"
#include "ax/optim2/linesearch/base.hpp"
#include "ax/optim2/problem.hpp"

namespace ax::optim2 {

Optimizer_GradientDescent::Optimizer_GradientDescent() {
  linesearch_ = std::make_unique<LineSearch_Backtracking>();
}

OptimizeResult Optimizer_GradientDescent::Optimize(OptimizeParam param) {
  AX_EXPECTS(problem_ != nullptr);
  AX_EXPECTS(linesearch_ != nullptr);
  linesearch_->SetProblem(problem_);

  OptimizeResult result;
  auto& iter = result.n_iter_;
  const size_t max_iter = param.max_iter_.value_or(max_iter_);
  const Real tol_grad = param.tol_grad_.value_or(tol_grad_);
  const Real tol_var = param.tol_var_.value_or(tol_var_);

  problem_->BeginOptimize();
  problem_->StepForwardActual();
  search_direction_ = ensure_buffer<Real>(search_direction_, problem_->GetVariables().Device(),
                                          problem_->GetVariables().Shape());

  for (iter = 0; iter < max_iter; ++iter) {
    problem_->OnStep(false, iter);

    // first, test the criteria.
    bool any_converged = false;
    if (problem_->GetGaridentNorm() < tol_grad) {
      result.converged_grad_ = true;
      any_converged = true;
    }
    if (iter > 0 && problem_->GetLastVariableChange() < tol_var) {
      result.converged_var_ = true;
      any_converged = true;
    }
    if (any_converged) {
      break;
    }

    // do step.
    auto sd = search_direction_->View();
    math::buffer_blas::copy(sd, problem_->GetGradient());
    math::buffer_blas::scal(-1.0, sd);

    LineSearchParam ls_param{.search_direction_ = sd};
    auto linesearch_result = linesearch_->Optimize(ls_param);
    if (!linesearch_result.converged_) {
      AX_ERROR("Line search failed.");
      break;
    }

    // commit the change due to line search.
    problem_->StepForwardActual();
  }

  problem_->EndOptimize();
  result.f_opt_ = problem_->GetEnergy();
  return result;
}

}  // namespace ax::optim2