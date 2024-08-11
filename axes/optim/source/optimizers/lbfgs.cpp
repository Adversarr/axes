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
real cosine_sim(Variable const& a, Variable const& b) {
  return math::dot(a, b) / (a.norm() * b.norm());
}

static Variable approx_solve_default(Variable const& r, Variable const& sk, Gradient const& yk) {
  if (sk.size() == 0 || yk.size() == 0) {
    return r;
  } else {
    real H0 = math::dot(sk, yk) / (math::norm2(yk) + math::epsilon<real>);
    return H0 * r;
  }
}

struct RotatingHistoryBuffer {
  explicit RotatingHistoryBuffer(size_t size)
      : s_(size), y_(size), rho_(size), alpha_(size), total_(size) {}

  void Reshape(idx rows, idx cols) {
    for (auto& v : s_) {
      v.resize(rows, cols);
    }
    for (auto& v : y_) {
      v.resize(rows, cols);
    }
  }

  void Push(Variable const& s, Gradient const& y, real rho) {
    const size_t end = pushed_size_ % total_;
    s_[end] = s;
    y_[end] = y;
    rho_[end] = rho;
    pushed_size_ += 1;
  }

  size_t ActualSubscript(size_t i) const {
    return pushed_size_ < total_ ? i : (pushed_size_ + i) % total_;
  }

  void TwoLoopPre(Gradient& q) const {
    const size_t available = std::min(pushed_size_, total_);
    for (size_t i = available; i > 0; i--) {
      const size_t rotate_id = ActualSubscript(i - 1);
      const Variable& si = s_[rotate_id];
      const Gradient& yi = y_[rotate_id];
      const real rho_i = rho_[rotate_id];
      alpha_[rotate_id] = rho_i * math::dot(si, q);
      q.noalias() -= alpha_[rotate_id] * yi;
    }
  }

  void TwoLoopPost(Variable& r) const {
    const size_t available = std::min(pushed_size_, total_);
    for (size_t i = 0; i < available; i++) {
      const size_t rotate_id = ActualSubscript(i);
      const Variable& si = s_[rotate_id];
      const Gradient& yi = y_[rotate_id];
      const real beta = rho_[rotate_id] * math::dot(yi, r);
      r.noalias() += si * (alpha_[rotate_id] - beta);
    }
  }

  Variable const& SBack() const {
    const size_t available = std::min(pushed_size_, total_);
    const size_t rotate_id = ActualSubscript(available - 1);
    return s_[rotate_id];
  }

  Gradient const& YBack() const {
    const size_t available = std::min(pushed_size_, total_);
    const size_t rotate_id = ActualSubscript(available - 1);
    return y_[rotate_id];
  }

  bool Empty() const { return pushed_size_ == 0; }

  size_t Size() const { return pushed_size_; }

  std::vector<Variable> s_;
  std::vector<Gradient> y_;
  std::vector<real> rho_;

  mutable std::vector<real> alpha_;
  size_t pushed_size_ = 0;
  const size_t total_;
};

OptResult Optimizer_Lbfgs::Optimize(OptProblem const& problem_, const Variable& x0) const {
  AX_THROW_IF_FALSE(problem_.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem_.HasEnergy(), "Energy function not set");
  AX_THROW_IF_LT(history_size_, 0, "Invalid history size: {}", history_size_);

  // SECT: Initialize
  idx rows = x0.rows(), cols = x0.cols();
  Variable x = x0;
  Gradient grad = problem_.EvalGrad(x);
  real f_iter = problem_.EvalEnergy(x);

  idx iter = 0;
  bool converged = false;
  bool converge_grad = false;
  bool converge_var = false;
  bool verbose = verbose_;
  RotatingHistoryBuffer bfgs(history_size_);
  bfgs.Reshape(rows, cols);

  Variable s_new(rows, cols);
  Gradient g_new(rows, cols);
  Gradient y_new(rows, cols);
  Variable x_old = x;

  SparseHessian sp_hessian;
  bool const check_approx_quality
      = check_approx_quality_ && problem_.HasSparseHessian() && approx_solve_;
  if (!check_approx_quality && check_approx_quality_) {
    AX_WARN(
        "Approx Hessian is not set or Hessian is not sparse, Approx quality check is disabled.");
  }

  auto check_quality
      = [&](std::string name, Variable const& residual, math::vecxr const& approx) {
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
    Gradient q = grad;
    Variable r = q;
    if (check_approx_quality) {
      sp_hessian = problem_.EvalSparseHessian(x);
    }

    if (!bfgs.Empty()) {
      // SECT: Limited Memory BFGS: Loop over history 1
      bfgs.TwoLoopPre(q);

      // SECT: Central step
      auto const& sback = bfgs.SBack();
      auto const& yback = bfgs.YBack();
      if (approx_solve_) {
        r = approx_solve_(q, sback, yback);
      } else {
        r = q;
      }

      if (check_approx_quality) {
        check_quality("Approx", q, r);
      }

      // SECT: Limited Memory BFGS: Loop over history 2
      bfgs.TwoLoopPost(r);
    } else if (approx_solve_) {
      r = approx_solve_(r, Variable(), Gradient());
      if (check_approx_quality) {
        check_quality("Approx", q, r);
      }
    }
    if (check_approx_quality) {
      check_quality("Whole BFGS", q, r);
    }
    Variable dir = -r;

    real const d_dot_grad = math::dot(dir, grad);
    if (d_dot_grad >= 0.) {
      AX_ERROR("L-BFGS: Direction may not descent: value={} Early break! |r|={}", d_dot_grad,
          math::norm(dir));
      break;
    }


    // SECT: Line search
    real f_opt;
    auto ls_result = linesearch_->Optimize(problem_, x, grad, dir);
    Variable& x_opt = ls_result.x_opt_;

    s_new = x_opt - x;
    std::swap(x, x_old);
    x = std::move(x_opt);
    f_opt = problem_.EvalEnergy(x);
    g_new = problem_.EvalGrad(x);
    y_new = g_new - grad;
    f_iter = f_opt;
    grad = g_new;

    if (real const rho_current = 1.0 / (math::dot(s_new, y_new) + 1e-19); rho_current <= 0) {
      // Last History is not available, discard it in memory
      AX_WARN(
          "LBFGS {}: rho is not positive: {} the problem is too stiff or the "
          "inverse approximation is bad. Consider use a better linesearch to guarantee the "
          "strong wolfe condition.",
          iter, rho_current);
    } else {
      // Last History is available.
      bfgs.Push(s_new, y_new, rho_current);
    }
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

  auto* ls = static_cast<Linesearch_Backtracking*>(linesearch_.get());
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
