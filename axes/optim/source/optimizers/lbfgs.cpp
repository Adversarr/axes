#include "ax/optim/optimizers/lbfgs.hpp"

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/formatting.hpp"
#include "ax/math/linsys/common.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/time.hpp"

namespace ax::optim {

real cosine_sim(math::vecxr const& a, math::vecxr const& b) {
  return a.dot(b) / (a.norm() * b.norm());
}

static math::vecxr approx_solve_default(math::vecxr const& r, math::vecxr const& sk,
                                        math::vecxr const& yk) {
  if (sk.size() == 0 || yk.size() == 0) {
    return r;
  } else {
    real H0 = sk.dot(yk) / (yk.dot(yk) + math::epsilon<real>);
    return H0 * r;
  }
}

OptResult Optimizer_Lbfgs::Optimize(OptProblem const& problem_, math::vecxr const& x0) const {
  AX_THROW_IF_FALSE(problem_.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem_.HasEnergy(), "Energy function not set");
  AX_THROW_IF_LT(history_size_, 0, "Invalid history size: {}", history_size_);

  // SECT: Initialize
  idx n_dof = x0.size();
  math::vecxr x = x0;
  math::vecxr grad = problem_.EvalGrad(x);
  real f_iter = problem_.EvalEnergy(x);

  idx iter = 0, history_in_use = 0;
  bool converged = false;
  bool converge_grad = false;
  bool converge_var = false;
  bool verbose = verbose_;
  math::vecxr alpha(history_size_);
  math::vecxr rho(history_size_);
  math::matxxr S(n_dof, history_size_);
  math::matxxr Y(n_dof, history_size_);

  math::vecxr s_new(n_dof);
  math::vecxr g_new(n_dof);
  math::vecxr y_new(n_dof);
  math::vecxr x_old = x;

  math::spmatr sp_hessian;
  bool const check_approx_quality
      = check_approx_quality_ && problem_.HasSparseHessian() && approx_solve_;
  if (!check_approx_quality && check_approx_quality_) {
    AX_WARN(
        "Approx Hessian is not set or Hessian is not sparse, Approx quality check is disabled.");
  }

  auto check_quality
      = [&](std::string name, math::vecxr const& residual, math::vecxr const& approx) {
          math::SparseSolver_ConjugateGradient cg;
          cg.SetProblem(sp_hessian).Compute();
          auto result = cg.Solve(residual, approx);
          if (!result.converged_) {
            // AX_LOG(ERROR) << name << ": CG failed to converge for check quality.";
            AX_ERROR("{}: CG failed to converge for check quality.", name);
            return;
          }
          real cs = cosine_sim(approx, result.solution_);
          real l2 = (approx - result.solution_).norm();
          real rel = l2 / result.solution_.norm();
          AX_ERROR("{}: cs={} l2={} rel={}", name, cs, l2, rel);
        };

  while (iter < max_iter_) {
    // SECT: Verbose
    problem_.EvalVerbose(iter, x, f_iter);
    if (verbose) {
      AX_INFO("L-BFGS iter {}:\n  x: {}\n  f: {}\n  grad: {}", iter, x, f_iter, grad);
    }

    AX_THROW_IF_FALSE(math::isfinite(f_iter), "LBFGS: Energy function returns Infinite number!");

    // SECT: Check convergence
    converge_grad = problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, grad) < tol_grad_;
    converge_var
        = iter > 1 && problem_.HasConvergeVar() && problem_.EvalConvergeVar(x, x_old) < tol_var_;
    if (converge_grad || converge_var) {
      converged = true;
      break;
    }

    // SECT: LBFGS Two Loop
    math::vecxr q = grad;
    math::vecxr r = q;
    idx const available_history = std::min(history_in_use, history_size_);
    if (check_approx_quality) {
      sp_hessian = problem_.EvalSparseHessian(x);
    }

    if (available_history > 0) {
      // SECT: Limited Memory BFGS: Loop over history 1
      // AX_TIMEIT("Two Loop");
      for (idx i = available_history - 1; i >= 0; i--) {
        idx rotate_id = (history_in_use + i) % available_history;
        auto const& si = S.col(rotate_id);
        auto const& yi = Y.col(rotate_id);
        real rho_i = rho[rotate_id];
        real alpha_i = (alpha[i] = rho_i * si.dot(q));
        q.noalias() -= alpha_i * yi;
      }
      // SECT: Central step
      idx rotate_id = (history_in_use + available_history - 1) % available_history;
      auto const& sback = S.col(rotate_id);
      auto const& yback = Y.col(rotate_id);
      r = approx_solve_(q, sback, yback);

      if (check_approx_quality) {
        check_quality("Approx", q, r);
      }

      // SECT: Limited Memory BFGS: Loop over history 2
      for (idx i = 0; i < available_history; i++) {
        idx rotate_id = (history_in_use + i) % available_history;
        auto const& si = S.col(rotate_id);
        auto const& yi = Y.col(rotate_id);
        real beta = rho[rotate_id] * yi.dot(r);
        r.noalias() += si * (alpha[i] - beta);
      }
    } else if (approx_solve_) {
      r = approx_solve_(r, math::vecxr(), math::vecxr());
      if (check_approx_quality) {
        check_quality("Approx", q, r);
      }
    }
    if (check_approx_quality) {
      check_quality("Whole BFGS", q, r);
    }
    math::vecxr dir = -r;

    real const d_dot_grad = math::dot(dir, grad);
    if (d_dot_grad >= real(0)) {
      // AX_LOG(ERROR) << "L-BFGS: Direction may not descent: value=" << d_dot_grad << "Early break!";
      AX_ERROR("L-BFGS: Direction may not descent: value={} Early break!", d_dot_grad);
      break;
    }

    // SECT: Line search
    math::vecxr x_opt;
    real f_opt;
    try {
      auto ls_result = linesearch_->Optimize(problem_, x, grad, dir);
      x_opt = std::move(ls_result.x_opt_);
      f_opt = ls_result.f_opt_;
    } catch (...) {
      std::throw_with_nested(std::runtime_error("LBFGS: Line search failed."));
    }
    s_new.noalias() = x_opt - x;
    g_new.noalias() = problem_.EvalGrad(x_opt);
    y_new.noalias() = g_new - grad;

    f_iter = f_opt;
    std::swap(x, x_old);
    x = std::move(x_opt);
    S.col(iter % history_size_) = s_new;
    Y.col(iter % history_size_) = y_new;
    real rho_current = (rho[iter % history_size_] = 1.0 / (math::dot(s_new, y_new) + 1e-19));
    if (rho_current <= 0) {
      // Last History is not available, discard it in memory
      AX_WARN("LBFGS: rho is not positive: {} history_in_use={} the problem is too stiff or the "
              "inverse approximation is bad. Consider use a better linesearch to guarantee the "
              "strong wolfe condition.",
              rho_current, history_in_use);
    } else {
      // Last History is available.
      history_in_use++;
    }
    grad.swap(g_new);
    iter++;
  }

  OptResult result;
  result.x_opt_ = x;
  result.f_opt_ = f_iter;
  result.converged_ = converged;
  result.converged_grad_ = converge_grad;
  result.converged_var_ = converge_var;
  result.n_iter_ = iter;

  return result;
}

Optimizer_Lbfgs::Optimizer_Lbfgs() {
  linesearch_ = std::make_unique<Linesearch_Backtracking>();

  auto *ls = reinterpret_cast<Linesearch_Backtracking*>(linesearch_.get());
  ls->required_descent_rate_ = 1e-4;
  ls->initial_step_length_ = 1.0;
  ls->step_shrink_rate_ = 0.7;

  SetApproxSolve(approx_solve_default);
}

void Optimizer_Lbfgs::SetOptions(utils::Options const& options) {
  OptimizerBase::SetOptions(options);
  AX_SYNC_OPT_IF(options, idx, history_size) {
    AX_THROW_IF_LT(history_size_, 0, "History size should be positive");
  }
  AX_SYNC_OPT(options, bool, check_approx_quality);
  utils::extract_and_create<LinesearchBase, LineSearchKind>(options, "linesearch", linesearch_);
  utils::extract_tunable(options, "linesearch_opt", linesearch_.get());
  OptimizerBase::SetOptions(options);
}

utils::Options Optimizer_Lbfgs::GetOptions() const {
  utils::Options opt = OptimizerBase::GetOptions();
  opt["history_size"] = history_size_;
  opt["linesearch"] = utils::reflect_name(linesearch_->GetKind()).value();
  opt["linesearch_opt"] = linesearch_->GetOptions();
  opt["check_approx_quality"] = check_approx_quality_;
  return opt;
}

void Optimizer_Lbfgs::SetApproxSolve(LbfgsHessianApproximator approximator) {
  approx_solve_ = approximator;
}

}  // namespace ax::optim
