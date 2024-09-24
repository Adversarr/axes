#define SPDLOG_ACTIVE_LEVEL 0
#include "ax/optim2/optimizer/ncg.hpp"

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/linesearch/backtrack.hpp"

namespace ax::optim2 {

Optimizer_NonlinearCg::Optimizer_NonlinearCg() {
  linesearch_ = std::make_unique<LineSearch_Backtracking>();
}

Optimizer_NonlinearCg::~Optimizer_NonlinearCg() = default;

OptimizerKind Optimizer_NonlinearCg::GetKind() const {
  return OptimizerKind::NonlinearCg;
}

OptimizeResult Optimizer_NonlinearCg::Optimize(OptimizeParam param) {
  AX_EXPECTS(problem_ != nullptr);
  AX_EXPECTS(linesearch_ != nullptr);
  linesearch_->SetProblem(problem_);

  OptimizeResult result;
  auto& iter = result.n_iter_;
  const size_t max_iter = param.max_iter_.value_or(max_iter_);

  // initialize the variables
  search_direction_ = ensure_buffer<Real>(search_direction_, problem_->GetVariables().Device(),
                                          problem_->GetVariables().Shape());
  prec_grad_ = ensure_buffer<Real>(prec_grad_, problem_->GetVariables().Device(),
                                   problem_->GetVariables().Shape());

  // prepare default preconditioner.
  auto default_precond = [](RealBufferView /* s */) {};

  if (!precond_) {
    precond_ = default_precond;
  }

  problem_->BeginOptimize();
  problem_->StepForwardActual();

  auto d = search_direction_->View();
  auto s = prec_grad_->View();
  LineSearchParam ls_param(d);
  // d <- - g
  auto grad = problem_->GetGradient();  // grad = grad[n]

  // set prec_grad
  math::buffer_blas::copy(s, grad);  // s <-          grad
  precond_(s);                       // s <-  M.inv * grad

  // Initialize d
  math::buffer_blas::copy(d, s);   // d <-  s
  math::buffer_blas::scal(-1, d);  // d <- -s

  Real delta_new = math::buffer_blas::dot(grad, s);

  for (iter = 0; iter < max_iter; ++iter) {
    problem_->OnStep(false, iter);
    if (TestCriteria(param, result)) {
      break;
    }

    // do line search.
    LineSearchResult ls_result;
    try {
      ls_result = linesearch_->Optimize(ls_param);
      if (!ls_result.converged_) {
        throw std::runtime_error("Line search failed.");
      }
    } catch (const std::runtime_error& e) {
      // restart from -s.
      math::buffer_blas::copy(d, s);
      math::buffer_blas::scal(-1, d);
      ls_result = linesearch_->Optimize(ls_param);
      if (!ls_result.converged_) {
        AX_CRITICAL("Line search failed twice, force break!");
        break;
      }
    }

    AX_TRACE("Line search converged at iter={} with step_size={}", ls_result.n_iter_,
             ls_result.step_length_);

    // use gradient to update search direction.
    problem_->UpdateGradient();                        //     grad = grad[n]
    Real delta_old = delta_new;                        // grad[n-1] dot s[n-1]
    Real delta_mid = math::buffer_blas::dot(grad, s);  //   grad[n] dot s[n-1]

    // update s and do preconditioning.
    math::buffer_blas::copy(s, grad);             //     s[n] = grad[n]
    precond_(s);                                  //     s[n] = M.inv * grad[n]
    delta_new = math::buffer_blas::dot(grad, s);  //  grad[n] dot s[n]

    Real expect_descent = linesearch_->OriginalGradientDotDirection();
    Real beta = math::nan<>;  // beta, nan init.
    switch (strategy_) {
      case NonlinearCgStrategy::FletcherReeves: {
        beta = delta_new / delta_old;
        break;
      }
      case NonlinearCgStrategy::PolakRibiere: {
        beta = (delta_new - delta_mid) / delta_old;
        break;
      }
      case NonlinearCgStrategy::PolakRibiereClamped: {
        beta = std::max(0.0, (delta_new - delta_mid) / delta_old);
        break;
      }

        // In following cases: ed_mid = grad[n] dot search_dir[n-1]

      case NonlinearCgStrategy::HestenesStiefel: {
        Real ed_mid = math::buffer_blas::dot(grad, d);
        beta = (delta_new - delta_mid) / (ed_mid - expect_descent);
        break;
      }

      case NonlinearCgStrategy::HestenesStiefelClamped: {
        Real ed_mid = math::buffer_blas::dot(grad, d);
        beta = std::max(0.0, (delta_new - delta_mid) / (ed_mid - expect_descent));
        break;
      }

      case NonlinearCgStrategy::DaiYuan: {
        Real ed_mid = math::buffer_blas::dot(grad, d);
        beta = delta_new / (ed_mid - expect_descent);
        break;
      }

      default:
        AX_UNREACHABLE();
    }

    // update search direction.
    math::buffer_blas::scal(beta, d);     // d <- beta * d
    math::buffer_blas::axpy(-1.0, s, d);  // d <- -s + beta * d

    AX_TRACE("beta: {:12.6e} <s,g>={:12.6e}", beta, delta_new);
    // update variables.
    problem_->StepForwardActual();

    if ((iter + 1) % restart_period_ == 0) {
      AX_TRACE("Restart at iter={}", iter);
      math::buffer_blas::copy(d, s);
      math::buffer_blas::scal(-1, d);
    }
  }

  problem_->EndOptimize();
  result.f_opt_ = problem_->GetEnergy();
  return result;
}

}  // namespace ax::optim2