// #define SPDLOG_ACTIVE_LEVEL 0
#include "ax/optim2/optimizer/lbfgs.hpp"

#include "ax/core/buffer/create_buffer.hpp"
#include "ax/math/buffer_blas.hpp"
#include "ax/optim2/linesearch/backtrack.hpp"
#include "ax/optim2/linesearch/base.hpp"
#include "ax/optim2/linesearch/wolfe.hpp"

namespace ax::optim2 {

Optimizer_LBFGS::Optimizer_LBFGS() {
  linesearch_ = std::make_unique<LineSearch_Wolfe>();
}

Optimizer_LBFGS::~Optimizer_LBFGS() = default;

OptimizeResult Optimizer_LBFGS::Optimize(OptimizeParam param) {
  AX_EXPECTS(problem_ != nullptr);
  AX_EXPECTS(linesearch_ != nullptr);
  CheckInputParam(param);
  if (!reuse_history_) {
    history_push_cnt_ = 0;
  }

  linesearch_->SetProblem(problem_);

  {  // setup the history buffers
    auto ensure = [example = problem_->GetVariables()](auto& ptr) {
      ptr = ensure_buffer<Real>(ptr, example.Device(), example.Shape());
    };
    if (s_.size() != history_size_) {
      s_.resize(history_size_);
      y_.resize(history_size_);
      rho_.resize(history_size_);
      alpha_.resize(history_size_);

      for (auto& ptr : s_) {
        ensure(ptr);
      }

      for (auto& ptr : y_) {
        ensure(ptr);
      }
    }
    ensure(search_direction_);
  }

  OptimizeResult result;
  problem_->BeginOptimize();
  problem_->StepForwardActual();
  auto final_act = finally([&] {
    problem_->EndOptimize();
    result.f_opt_ = problem_->GetEnergy();
  });
  const size_t max_iter = param.max_iter_.value_or(max_iter_);
  size_t& iter = result.n_iter_;

  auto d = search_direction_->View();
  auto grad = problem_->GetGradient();

  LineSearchParam ls_param(d);
  for (iter = 0; iter < max_iter; ++iter) {
    problem_->OnStep(false, iter);
    if (TestCriteria(param, result)) {
      break;
    }

    math::buffer_blas::copy(d, grad);  // d <-        grad
    SolveApproximator(iter, d);        // d <-  H_k * grad
    math::buffer_blas::scal(-1, d);    // d <- -H_k * grad

    // do line search
    LineSearchResult ls_result;
    try {
      ls_result = linesearch_->Optimize(ls_param);
    } catch (const std::runtime_error& rte) {
      AX_ERROR("Line Search Failed at iteration {:3}!!!", iter);
      throw;
    }
    if (!ls_result.converged_) {
      AX_ERROR("Line search failed.");
      break;
    }

    AX_TRACE("Line search converged at iter={} with step_size={}", ls_result.n_iter_,
             ls_result.step_length_);

    // push history.
    // currently, x = x_k+1, and x_backup = x_k.
    PushHistory(iter);

    // commit the change due to line search.
    problem_->StepForwardActual();
  }

  return result;
}

void Optimizer_LBFGS::PushHistory(size_t iter) {
  const size_t rotate_id = history_push_cnt_ % history_size_;
  auto& s = s_[rotate_id];
  auto& y = y_[rotate_id];
  auto& rho = rho_[rotate_id];
  AX_TRACE("Put into history: {}", rotate_id);
  auto [sv, yv] = make_view(s, y);
  problem_->UpdateGradient();
  math::buffer_blas::copy(sv, problem_->GetVariables());            // s <- x_k+1
  math::buffer_blas::axpy(-1, problem_->GetBackupVariables(), sv);  // s <- x_k+1 - x_k

  math::buffer_blas::copy(yv, problem_->GetGradient());            // y <- grad_k+1
  math::buffer_blas::axpy(-1, problem_->GetBackupGradient(), yv);  // y <- grad_k+1 - grad_k

  rho = 1.0 / (math::buffer_blas::dot(sv, yv) + 1e-20);

  if (rho <= 0 || !math::isfinite(rho)) {
    AX_THROW_RUNTIME_ERROR(
        "LBFGS {}: rho is not positive: {} the problem is too stiff or the "
        "inverse approximation is bad. Consider use a better linesearch to guarantee the "
        "strong wolfe condition.",
        iter, rho);
  }

  AX_TRACE("|s|= {:12.6e} |y|= {:12.6e}", math::buffer_blas::norm(sv), math::buffer_blas::norm(yv));
  ++history_push_cnt_;
}

// Implements Algorithm 7.4 (L-BFGS two-loop recursion).
void Optimizer_LBFGS::SolveApproximator(size_t iter, RealBufferView q) {
  const size_t available = std::min(history_push_cnt_, history_size_);
  // pre
  for (size_t i = 1; i <= available; i++) {  // k-1 to k-m
    const size_t rotate_id = (history_push_cnt_ + history_size_ - i) % history_size_;
    AX_TRACE("(Pre) Load history: {}", rotate_id);
    const Real si_q = math::buffer_blas::dot(s_[rotate_id]->ConstView(), q);
    const Real alpha = rho_[rotate_id] * si_q;
    alpha_[rotate_id] = alpha;

    math::buffer_blas::axpy(-alpha, y_[rotate_id]->ConstView(), q);  // q <- q - alpha * y
  }

  // central
  const size_t most_recent = (history_push_cnt_ + history_size_ - 1) % history_size_;
  const auto& last_s = s_[most_recent];
  const auto& last_y = y_[most_recent];
  AX_TRACE("Center using history: {}", most_recent);
  if (precond_) {
    precond_(q, last_s->ConstView(), last_y->ConstView(), available > 0);
  } else if (available > 0) {
    // do an approximation about the eigenvalue.
    // s_k y_k-1. / y_k-1 y_k-1.
    const Real sy = 1.0 / rho_[most_recent];
    const Real yy = math::buffer_blas::dot(last_y->ConstView(), last_y->ConstView());

    const Real gamma = sy / yy;
    math::buffer_blas::scal(gamma, q);
    AX_TRACE("gamma: {}", gamma);
  }

  auto r = q;  // r is just a alias of q.

  // post
  for (size_t i = available; i != 0; --i) {  // k-m to k-1
    const size_t rotate_id = (history_push_cnt_ + history_size_ - i) % history_size_;
    AX_TRACE("(Post) Load history: {}", rotate_id);
    auto [s, y] = make_view(s_[rotate_id], y_[rotate_id]);
    const Real yi_r = math::buffer_blas::dot(y, r);
    const Real beta = rho_[rotate_id] * yi_r;
    math::buffer_blas::axpy(alpha_[rotate_id] - beta, s, r);  // r <- r + (alpha - beta) * s
  }
}

}  // namespace ax::optim2