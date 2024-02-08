#include "axes/optim/optimizers/lbfgs.hpp"

#include "axes/core/echo.hpp"
#include "axes/optim/linesearch/backtracking.hpp"
#include "axes/optim/linesearch/linesearch.hpp"
#include "axes/utils/status.hpp"

namespace ax::optim {

OptResult Lbfgs::Optimize(OptProblem const& problem_, math::vecxr const& x0) {
  std::vector<math::vecxr> s;
  std::vector<math::vecxr> y;
  s.reserve(history_size_);
  y.reserve(history_size_);
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
  math::vecxr x = x0;
  math::vecxr g = problem_.EvalGrad(x);
  real f_iter = problem_.EvalEnergy(x);

  idx iter = 0;
  bool converged = false;
  bool converge_grad = false;
  bool converge_var = false;
  bool verbose = verbose_;
  while (iter < max_iter_) {
    // SECT: Verbose
    if (verbose) {
      problem_.EvalVerbose(iter, x, f_iter);
    }

    DLOG(INFO) << "L-BFGS iter " << iter << std::endl
               << "  x: " << x.transpose() << std::endl
               << "  f: " << f_iter << std::endl
               << "  grad: " << g.transpose();

    // SECT: Check convergence
    converge_grad = problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, g) < tol_grad_;
    converge_var
        = iter > 1 && problem_.HasConvergeVar() && problem_.EvalConvergeVar(x, x0) < tol_var_;
    if (converge_grad || converge_var) {
      converged = true;
      break;
    }

    // SECT: LBFGS Two Loop
    math::vecxr alpha(history_size_);
    math::vecxr rho(history_size_);
    math::vecxr q = g;
    math::vecxr r = q;
    if (!s.empty()) {
      for (idx i = s.size() - 1; i >= 0; i--) {
        real rho_i = (rho[i] = 1.0 / s[i].dot(y[i]));
        real alpha_i = (alpha[i] = rho_i * s[i].dot(q));
        q -= alpha_i * y[i];
      }
      real H0 = s.back().dot(y.back()) / y.back().dot(y.back());
      r = H0 * q;
      for (size_t i = 0; i < s.size(); i++) {
        real beta = rho[i] * y[i].dot(r);
        r += s[i] * (alpha[i] - beta);
      }
    }
    math::vecxr dir = -r;

    // SECT: Line search
    auto ls_result = linesearch_->Optimize(problem_, x, dir);
    if (!ls_result.ok()) {
      LOG(ERROR) << "Line search failed: " << ls_result.status();
      return ls_result.status();
    }

    math::vecxr s_new = ls_result->x_opt_ - x;
    math::vecxr g_new = problem_.EvalGrad(ls_result->x_opt_);
    math::vecxr y_new = g_new - g;

    if (s.size() == (size_t)history_size_) {
      s.erase(s.begin());
      y.erase(y.begin());
    }
    s.push_back(s_new);
    y.push_back(y_new);
    g = g_new;
    x = ls_result->x_opt_;
    iter++;
  }
  OptResultImpl result;
  result.x_opt_ = x;
  result.f_opt_ = problem_.EvalEnergy(x);
  result.converged_ = converged;
  result.converged_grad_ = converge_grad;
  result.converged_var_ = converge_var;
  result.n_iter_ = iter;

  return result;
}

Lbfgs::Lbfgs() { linesearch_ = std::make_unique<BacktrackingLinesearch>(); }

void Lbfgs::SetOptions(utils::Opt const& options) {
  OptimizerBase::SetOptions(options);
  AX_SYNC_OPT_(options, idx, history_size);
  AX_SYNC_OPT_(options, std::string, linesearch_name);
  if (options.Holds<utils::Opt>("linesearch_opt")) {
    linesearch_->SetOptions(options.Get<utils::Opt>("linesearch_opt"));
  }
}

utils::Opt Lbfgs::GetOptions() const {
  utils::Opt opt = OptimizerBase::GetOptions();
  opt["history_size"] = history_size_;
  opt["linesearch_name"] = linesearch_name_;
  opt["linesearch_opt"] = linesearch_->GetOptions();
  return opt;
}

}  // namespace ax::optim
