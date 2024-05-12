#include "ax/optim/optimizers/lbfgs.hpp"

#include "ax/core/echo.hpp"
#include "ax/math/linsys/common.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/status.hpp"
#include "ax/core/excepts.hpp"

#include "ax/utils/time.hpp"

// #define CHECK_APPROX_SOLVER


namespace ax::optim {

real cosine_sim(math::vecxr const& a, math::vecxr const& b) { return a.dot(b) / (a.norm() * b.norm());
}

OptResult Lbfgs::Optimize(OptProblem const& problem_, math::vecxr const& x0) const {
  AX_THROW_IF_FALSE(problem_.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem_.HasEnergy(), "Energy function not set");
  AX_THROW_IF_LT(history_size_, 0, "Invalid history size: " + std::to_string(history_size_));

  AX_TIME_FUNC();

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

  math::vecxr s_new(n_dof);
  math::vecxr g_new(n_dof);
  math::vecxr y_new(n_dof);
  math::vecxr x_old = x;

  math::sp_matxxr sp_hessian;
  bool check_approx_quality = check_approx_quality_ && problem_.HasSparseHessian() && approx_solve_;
  if (!check_approx_quality && check_approx_quality_) {
    AX_LOG(WARNING) << "Approx Hessian is not set or Hessian is not sparse, "
                    << "Approx quality check is disabled.";
  }


  while (iter < max_iter_) {
    // SECT: Verbose
    problem_.EvalVerbose(iter, x, f_iter);
    if (verbose) {
      AX_DLOG(INFO) << "L-BFGS iter " << iter << std::endl
                    << "  x: " << x.transpose() << std::endl
                    << "  f: " << f_iter << std::endl
                    << "  grad: " << grad.transpose();
    }

    AX_THROW_IF_FALSE(math::isfinite(f_iter), "Energy function returns Infinite number!");

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
    idx const available_history = std::min(iter, history_size_);


    if (check_approx_quality) {
      sp_hessian = problem_.EvalSparseHessian(x);
    }

    if (available_history > 0) {
      AX_TIMEIT("Two Loop");
      for (idx i = available_history - 1; i >= 0; i--) {
        idx rotate_id = (iter + i) % available_history;
        auto const& si = S.col(rotate_id);
        auto const& yi = Y.col(rotate_id);
        real rho_i = rho[rotate_id];
        real alpha_i = (alpha[i] = rho_i * si.dot(q));
        q.noalias() -= alpha_i * yi;
      }
      idx rotate_id = (iter + available_history - 1) % available_history;
      auto const& sback = S.col(rotate_id);
      auto const& yback = Y.col(rotate_id);
      if (approx_solve_) {
        AX_TIMEIT("Approx Solve");
        r = approx_solve_(q);
        if (sp_hessian.size() > 0 && check_approx_quality) {
          math::LinsysProblem_Sparse sp;
          sp.A_ = sp_hessian;
          sp.b_ = q;
          sp.converge_residual_ = [&] (math::vecxr const& x, math::vecxr const& g) {
            return problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, g) < tol_grad_;
          };
          math::SparseSolver_ConjugateGradient cg;
          auto result = cg.SolveProblem(sp, {});
          if (result.ok()) {
            auto cs = cosine_sim(r, result->solution_);
            auto l2 = (r - result->solution_).norm();
            AX_LOG(ERROR) << "Q-Sovle Approx: Cosine Sim=" << cs
                          << "\t 2-norm=" << l2
                          << "\t rel-l2=" << l2 / result->solution_.norm();
          }
        }
      } else {
        real H0 = sback.dot(yback) / (yback.dot(yback) + 1e-19);
        r = H0 * q;
      }
      for (idx i = 0; i < available_history; i++) {
        idx rotate_id = (iter + i) % available_history;
        auto const& si = S.col(rotate_id);
        auto const& yi = Y.col(rotate_id);
        real beta = rho[rotate_id] * yi.dot(r);
        // r = r + si * (alpha[i] - beta);
        r.noalias() += si * (alpha[i] - beta);
      }
    } else if (approx_solve_) {
      AX_TIMEIT("Two Loop");
      {
        AX_TIMEIT("Approx Solve");
        r = approx_solve_(r);
      }
    }

    if (sp_hessian.size() > 0 && check_approx_quality) {
      math::LinsysProblem_Sparse sp;
      sp.A_ = sp_hessian;
      sp.b_ = grad;
      sp.converge_residual_ = [&] (math::vecxr const& x, math::vecxr const& g) {
        return problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, g) < tol_grad_;
      };
      math::SparseSolver_ConjugateGradient cg;
      auto result = cg.SolveProblem(sp, {});
      if (result.ok()) {
        auto cs = cosine_sim(r, result->solution_);
        auto l2 = (r - result->solution_).norm();
        AX_LOG(ERROR) << "L-BFGS Approx: Cosine Sim=" << cs
                      << "\t 2-norm=" << l2
                      << "\t rel-l2=" << l2 / result->solution_.norm();
      }
    }

    math::vecxr dir = -r;

    AX_THROW_IF_TRUE(math::dot(dir, grad) >= real(0), "Direction is not descent: " + std::to_string(math::dot(dir, grad)));
    // SECT: Line search
    auto ls_result = linesearch_->Optimize(problem_, x, grad, dir);

    s_new.noalias() = ls_result.x_opt_ - x;
    g_new.noalias() = problem_.EvalGrad(ls_result.x_opt_);
    y_new.noalias() = g_new - grad;

    f_iter = ls_result.f_opt_;
    std::swap(x, x_old);
    x = std::move(ls_result.x_opt_);
    S.col(iter % history_size_) = s_new;
    Y.col(iter % history_size_) = y_new;
    rho[iter % history_size_] = 1.0 / (math::dot(s_new, y_new) + 1e-19);
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

Lbfgs::Lbfgs() {
  linesearch_ = std::make_unique<Linesearch_Backtracking>();
  linesearch_name_ = "kBacktracking";

  auto ls = reinterpret_cast<Linesearch_Backtracking*>(linesearch_.get());
  ls->required_descent_rate_ = 1e-4;
  ls->initial_step_length_ = 1.0;
  ls->step_shrink_rate_ = 0.7;
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
