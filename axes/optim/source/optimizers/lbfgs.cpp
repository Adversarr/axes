#include "ax/optim/optimizers/lbfgs.hpp"

#include "ax/core/echo.hpp"
#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/status.hpp"

namespace ax::optim {

OptResult Lbfgs::Optimize(OptProblem const& problem_, math::vecxr const& x0) const {
  if (!problem_.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  }
  if (!problem_.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }

  if (history_size_ < 0) {
    return utils::InvalidArgumentError("Invalid history size: " + std::to_string(history_size_));
  }

  // SECT: Initialize
  idx n_dof = x0.size();
  math::vecxr x = x0;
  math::vecxr grad = problem_.EvalGrad(x);
  real f_iter = problem_.EvalEnergy(x);

  idx iter = 0;
  bool converged = false;
  bool converge_grad = false;
  bool converge_var = false;
  bool verbose = verbose_;
  math::vecxr alpha(history_size_);
  math::vecxr rho(history_size_);
  math::matxxr S(n_dof, history_size_);
  math::matxxr Y(n_dof, history_size_);

  while (iter < max_iter_) {
    // SECT: Verbose
    problem_.EvalVerbose(iter, x, f_iter);
    if (verbose) {
      AX_DLOG(INFO) << "L-BFGS iter " << iter << std::endl
                    << "  x: " << x.transpose() << std::endl
                    << "  f: " << f_iter << std::endl
                    << "  grad: " << grad.transpose();
    }

    if (!math::isfinite(f_iter)) {
      return utils::FailedPreconditionError("Energy function returns Infinite number!");
    }

    // SECT: Check convergence
    converge_grad = problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, grad) < tol_grad_;
    converge_var
        = iter > 1 && problem_.HasConvergeVar() && problem_.EvalConvergeVar(x, x0) < tol_var_;
    if (converge_grad || converge_var) {
      converged = true;
      break;
    }

    // SECT: LBFGS Two Loop
    math::vecxr q = grad;
    math::vecxr r = q;
    idx const available_history = std::min(iter, history_size_);

    if (available_history > 0) {
      for (idx i = available_history - 1; i >= 0; i--) {
        idx rotate_id = (iter + i) % available_history;
        auto const& si = S.col(rotate_id);
        auto const& yi = Y.col(rotate_id);
        real rho_i = rho[rotate_id];
        real alpha_i = (alpha[i] = rho_i * si.dot(q));
        q = q - alpha_i * yi;
      }
      idx rotate_id = (iter + available_history - 1) % available_history;
      auto const& sback = S.col(rotate_id);
      auto const& yback = Y.col(rotate_id);
      if (approx_solve_) {
        r = approx_solve_(r);
      } else {
        real H0 = sback.dot(yback) / (yback.dot(yback) + 1e-19);
        r = H0 * q;
      }
      for (idx i = 0; i < available_history; i++) {
        idx rotate_id = (iter + i) % available_history;
        auto const& si = S.col(rotate_id);
        auto const& yi = Y.col(rotate_id);
        real beta = rho[rotate_id] * yi.dot(r);
        r = r + si * (alpha[i] - beta);
      }
    } else if (approx_solve_) {
      r = approx_solve_(r);
    }
    math::vecxr dir = -r;

    if (math::dot(dir, grad) >= real(0)) {
      AX_LOG(ERROR) << "Direction is not descent: " << math::dot(dir, grad);
      return utils::FailedPreconditionError("Direction is not descent: Your Approx Hessian is not valid.");
    }

    // SECT: Line search
    auto ls_result = linesearch_->Optimize(problem_, x, dir);
    if (!ls_result.ok()) {
      AX_LOG(ERROR) << "Line search failed: " << ls_result.status();
      AX_LOG(ERROR) << "at Iteration" << iter;
      return ls_result.status();
    }

    math::vecxr s_new = ls_result->x_opt_ - x;
    math::vecxr g_new = problem_.EvalGrad(ls_result->x_opt_);
    math::vecxr y_new = g_new - grad;

    f_iter = ls_result->f_opt_;
    x = ls_result->x_opt_;
    S.col(iter % history_size_) = s_new;
    Y.col(iter % history_size_) = y_new;
    rho[iter % history_size_] = 1.0 / (math::dot(s_new, y_new) + 1e-19);
    grad = g_new;
    iter++;
  }
  OptResultImpl result;
  result.x_opt_ = x;
  result.f_opt_ = f_iter;
  result.converged_ = converged;
  result.converged_grad_ = converge_grad;
  result.converged_var_ = converge_var;
  result.n_iter_ = iter;

  return result;
}

Lbfgs::Lbfgs() {
  linesearch_ = std::make_unique<BacktrackingLinesearch>();
  linesearch_name_ = "kBacktracking";

  auto ls = reinterpret_cast<BacktrackingLinesearch*>(linesearch_.get());
  ls->c_ = 1e-4;
  ls->alpha_ = 1.0;
  ls->rho_ = 0.7;
}

Status Lbfgs::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT(options, idx, history_size);
  AX_SYNC_OPT_IF(options, std::string, linesearch_name) {
    auto ls = utils::reflect_enum<LineSearchKind>(linesearch_name_);
    AX_CHECK(ls) << "Unknown linesearch_name: " << linesearch_name_;
    linesearch_ = LinesearchBase::Create(ls.value());
    AX_RETURN_NOTOK_OR(utils::sync_to_field(*linesearch_, options, "linesearch_opt"));
  }
  return OptimizerBase::SetOptions(options);
}

utils::Opt Lbfgs::GetOptions() const {
  utils::Opt opt = OptimizerBase::GetOptions();
  opt["history_size"] = history_size_;
  opt["linesearch_name"] = linesearch_name_;
  opt["linesearch_opt"] = linesearch_->GetOptions();
  return opt;
}

void Lbfgs::SetApproxSolve(std::function<math::vecxr(math::vecxr const&)> hessian_approximation) {
  approx_solve_ = hessian_approximation;
}

}  // namespace ax::optim
