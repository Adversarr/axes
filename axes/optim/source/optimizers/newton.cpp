#include "ax/optim/optimizers/newton.hpp"

#include "ax/core/logging.hpp"
#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/dense/LLT.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/linsys/sparse/Cholmod.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/time.hpp"

namespace ax::optim {

OptResult Optimizer_Newton::Optimize(OptProblem const& problem_, Variable const& x0) const {
  AX_TIME_FUNC();
  AX_THROW_IF_FALSE(problem_.HasEnergy(), "Energy function not set");
  AX_THROW_IF_FALSE(problem_.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem_.HasHessian() || problem_.HasSparseHessian(),
                    "Hessian function not set");
  bool is_dense_hessian = problem_.HasHessian();

  // SECT: Initialize
  Variable x = x0;
  Real f_iter = problem_.EvalEnergy(x);
  Gradient grad = problem_.EvalGrad(x);
  Variable dir = -grad;

  Index iter = 0;
  bool converged = false;

  OptResult result;
  bool converge_grad = false;
  bool converge_var = false;
  // Main loop
  for (iter = 0; iter < max_iter_; ++iter) {
    // SECT: verbose_
    problem_.EvalVerbose(iter, x, f_iter);
    if (verbose_) {
      AX_INFO("Newton iter {:3}: f={:12.6e}  |g|={:12.6e}", iter, f_iter, math::norm(grad));
    }

    AX_THROW_IF_FALSE(math::isfinite(f_iter), "Energy function returns NaN or Inf");

    // SECT: Check convergence
    converge_grad = problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, grad) < tol_grad_;
    converge_var
        = iter > 1 && problem_.HasConvergeVar() && problem_.EvalConvergeVar(x, x0) < tol_var_;
    if (converge_grad || converge_var) {
      converged = true;
      break;
    }

    // SECT: Find a Dir
    if (is_dense_hessian) {
      auto H = problem_.EvalHessian(x);
      AX_THROW_IF_TRUE(H.rows() != x.rows() || H.cols() != x.rows(),
                       "Hessian matrix size mismatch");
      // TODO: this is not correct
      dense_solver_->SetProblem(H).Compute();
      dir = dense_solver_->Solve(-grad);
    } else {
      AX_TIMEIT("Eval and Solve Sparse System");
      auto H = problem_.EvalSparseHessian(x);
      AX_THROW_IF_TRUE(H.rows() != x.rows() || H.cols() != x.rows(),
                       "Hessian matrix size mismatch");
      auto prob = math::make_sparse_problem(H);
      if (problem_.HasConvergeGrad()) {
        prob->converge_residual_ = [&](Variable const& x, Gradient const& r) -> bool {
          return problem_.EvalConvergeGrad(x, r) < tol_grad_;
        };
      }
      if (problem_.HasConvergeVar()) {
        prob->converge_solution_ = [&](Variable const& x) -> bool {
          return problem_.EvalConvergeVar(x, x0) < tol_var_;
        };
      }
      sparse_solver_->SetProblem(std::move(prob)).Compute();
      auto solution = sparse_solver_->Solve(grad);
      dir = -solution.solution_;
    }

    // SECT: Line search
    const Real f_last = f_iter;
    if (linesearch_) {
      OptResult ls_result = linesearch_->Optimize(problem_, x, grad, dir);
      if (!ls_result.converged_) {
        AX_ERROR("Line Search Error: Failed to converge");
        break;
      }
      if (verbose_) {
        AX_INFO("linesearch: iter={:3}, step_length={:12.6e}", ls_result.n_iter_,
                ls_result.step_length_);
      }
      x = ls_result.x_opt_;
      f_iter = ls_result.f_opt_;
    } else {
      if (const Real dir_dot_grad = math::dot(dir, grad); dir_dot_grad > math::epsilon<Real>) {
        AX_ERROR("Direction is not descenting.");
        break;
      }

      x += dir;
      f_iter = problem_.EvalEnergy(x);
    }
    if (f_iter > f_last) {
      AX_ERROR("Line Search Error: Energy increased, last={}, current={}", f_last, f_iter);
      break;
    }
    grad = problem_.EvalGrad(x);
  }

  result.converged_grad_ = converge_grad;
  result.converged_var_ = converge_var;
  result.converged_ = converged;
  result.x_opt_ = x;
  result.f_opt_ = f_iter;
  result.n_iter_ = iter;
  return result;
}

Optimizer_Newton::Optimizer_Newton() {
  dense_solver_ = std::make_unique<math::DenseSolver_LLT>();
#ifdef AX_HAS_CHOLMOD
  sparse_solver_ = std::make_unique<math::SparseSolver_Cholmod>();
#else
  sparse_solver_ = std::make_unique<math::SparseSolver_ConjugateGradient>();
#endif
  linesearch_ = std::make_unique<Linesearch_Backtracking>();
}

void Optimizer_Newton::SetOptions(utils::Options const& options) {
  using namespace math;
  utils::extract_and_create<LinesearchBase, LineSearchKind>(options, "linesearch", linesearch_);
  utils::extract_and_create<DenseSolverBase, DenseSolverKind>(options, "dense_solver",
                                                              dense_solver_);
  utils::extract_and_create<SparseSolverBase, SparseSolverKind>(options, "sparse_solver",
                                                                sparse_solver_);

  utils::extract_tunable(options, "linesearch_opt", linesearch_.get());
  utils::extract_tunable(options, "dense_solver_opt", dense_solver_.get());
  utils::extract_tunable(options, "sparse_solver_opt", sparse_solver_.get());

  OptimizerBase::SetOptions(options);
}

utils::Options Optimizer_Newton::GetOptions() const {
  utils::Options opt = OptimizerBase::GetOptions();
  opt["linesearch"] = utils::reflect_name(linesearch_->GetKind()).value();
  opt["dense_solver"] = utils::reflect_name(dense_solver_->GetKind()).value();
  opt["sparse_solver"] = utils::reflect_name(sparse_solver_->GetKind()).value();
  opt["dense_solver_opt"] = dense_solver_->GetOptions();
  opt["sparse_solver_opt"] = sparse_solver_->GetOptions();
  return opt;
}

}  // namespace ax::optim
