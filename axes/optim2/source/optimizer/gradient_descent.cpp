#define SPDLOG_ACTIVE_LEVEL 0
#include "ax/optim2/optimizer/gradient_descent.hpp"

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/linesearch/backtrack.hpp"
#include "ax/optim2/linesearch/base.hpp"

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

  problem_->BeginOptimize();
  problem_->StepForwardActual();
  search_direction_ = ensure_buffer<Real>(search_direction_, problem_->GetVariables().Device(),
                                          problem_->GetVariables().Shape());
  auto sd = search_direction_->View();
  LineSearchParam ls_param(sd);

  for (iter = 0; iter < max_iter; ++iter) {
    problem_->OnStep(false, iter);

    // first, test the criteria.
    if (TestCriteria(param, result)) {
      break;
    }

    // do step.
    math::buffer_blas::copy(sd, problem_->GetGradient());
    math::buffer_blas::scal(-1.0, sd);

    AX_EXPECTS(math::buffer_blas::dot(sd, problem_->GetGradient()) < 0);
    auto linesearch_result = linesearch_->Optimize(ls_param);
    if (!linesearch_result.converged_) {
      AX_ERROR("Line search failed.");
      break;
    }

    AX_TRACE("Line search converged at iter={} with step_size={}", linesearch_result.n_iter_,
             linesearch_result.step_length_);

    // commit the change due to line search.
    problem_->StepForwardActual();
  }

  problem_->EndOptimize();
  result.f_opt_ = problem_->GetEnergy();
  return result;
}

}  // namespace ax::optim2