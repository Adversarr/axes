#include "ax/optim2/optimizer/newton.hpp"

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"
#include "ax/optim2/linesearch/backtrack.hpp"
#include "ax/optim2/linesearch/base.hpp"

namespace ax::optim2 {

Optimizer_Newton::Optimizer_Newton() {
  linear_solver_ = std::make_unique<math::GeneralSparseSolver_ConjugateGradient>();
  linesearch_ = std::make_unique<LineSearch_Backtracking>();
}

Optimizer_Newton::~Optimizer_Newton() = default;

OptimizeResult Optimizer_Newton::Optimize(OptimizeParam param) {
  AX_EXPECTS(problem_);
  AX_EXPECTS(linear_solver_);
  AX_EXPECTS(linesearch_);
  CheckInputParam(param);
  linesearch_->SetProblem(problem_);

  OptimizeResult result;
  auto& iter = result.n_iter_;
  const size_t max_iter = param.max_iter_.value_or(max_iter_);

  problem_->BeginOptimize();
  problem_->StepForwardActual();
  problem_->UpdateHessian();
  search_direction_ = ensure_buffer<Real>(search_direction_, problem_->GetVariables().Device(),
                                          problem_->GetVariables().Shape());
  auto sd = search_direction_->View();
  LineSearchParam ls_param(sd);
  linear_solver_->SetProblem(problem_->GetHessian());
  linear_solver_->Compute();
  auto grad = problem_->GetGradient();

  for (iter = 0; iter < max_iter; ++iter) {
    problem_->OnStep(false, iter);
    if (iter > 1) {
      // the first iteration, we have already updated the hessian.
      problem_->UpdateHessian();
    }

    // first, test the criteria.
    if (TestCriteria(param, result)) {
      break;
    }

    // do step.
    search_direction_->SetBytes(0);
    if (problem_->WillHessianChangeTopo()) {
      // recompute the linear solver.
      linear_solver_->SetProblem(problem_->GetHessian());
      linear_solver_->Compute();
    } else {
      // just do factorize.
      linear_solver_->Factorize();
    }

    linear_solver_->Solve(flatten(grad), flatten(sd));
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

  return result;
}

OptimizerKind Optimizer_Newton::GetKind() const {
  return OptimizerKind::Newton;
}
}  // namespace ax::optim2