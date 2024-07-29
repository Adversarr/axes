#include "ax/optim/optimizers/newton.hpp"

#include "ax/core/logging.hpp"
#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/dense/LLT.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/math/formatting.hpp"
#include "ax/utils/time.hpp"

namespace ax::optim {

OptResult Optimizer_Newton::Optimize(OptProblem const& problem_, math::vecxr const& x0) const {
  AX_TIME_FUNC();
  AX_THROW_IF_FALSE(problem_.HasEnergy(), "Energy function not set");
  AX_THROW_IF_FALSE(problem_.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem_.HasHessian() || problem_.HasSparseHessian(),
                    "Hessian function not set");
  bool is_dense_hessian = problem_.HasHessian();

  // SECT: Initialize
  math::vecxr x = x0;
  real f_iter = problem_.EvalEnergy(x);
  math::vecxr grad = problem_.EvalGrad(x);
  math::vecxr dir = -grad;

  idx iter = 0;
  bool converged = false;

  OptResult result;
  bool converge_grad = false;
  bool converge_var = false;
  // Main loop
  for (iter = 0; iter < max_iter_; ++iter) {
    // SECT: verbose_
    problem_.EvalVerbose(iter, x, f_iter);
    if (verbose_) {
      AX_INFO("Newton iter {}:\n  x: {}\n  f: {}\n  grad: {}", iter, x.transpose(), f_iter,
              grad.transpose());
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
      math::matxxr H = problem_.EvalHessian(x);
      AX_THROW_IF_TRUE(H.rows() != x.rows() || H.cols() != x.rows(),
                       "Hessian matrix size mismatch");
      // TODO: this is not correct
      dense_solver_->SetProblem(H).Compute();
      dir = dense_solver_->Solve(-grad);
    } else {
      AX_TIMEIT("Eval and Solve Sparse System");
      math::spmatr H = problem_.EvalSparseHessian(x);
      AX_THROW_IF_TRUE(H.rows() != x.rows() || H.cols() != x.rows(),
                       "Hessian matrix size mismatch");
      auto prob = math::make_sparse_problem(H);
      if (problem_.HasConvergeGrad()) {
        prob->converge_residual_ = [&](math::vecxr const& x, math::vecxr const& r) -> bool {
          return problem_.EvalConvergeGrad(x, r) < tol_grad_;
        };
      }
      if (problem_.HasConvergeVar()) {
        prob->converge_solution_ = [&](math::vecxr const& x) -> bool {
          return problem_.EvalConvergeVar(x, x0) < tol_var_;
        };
      }
      sparse_solver_->SetProblem(std::move(prob)).Compute();
      auto solution = sparse_solver_->Solve(grad);
      dir = -solution.solution_;
    }

    real dir_dot_grad = dir.dot(grad);
    if (dir_dot_grad > 0) {
      AX_ERROR("Direction is not descenting.");
      break;
    }

    // SECT: Line search
    OptResult ls_result = linesearch_->Optimize(problem_, x, grad, dir);
    x = ls_result.x_opt_;
    real f_last = f_iter;
    f_iter = ls_result.f_opt_;
    if (f_iter > f_last) {
      // AX_LOG(ERROR) << "Line Search Error: Energy increased!!!";
      AX_ERROR("Line Search Error: Energy increased, last={}, current={}", f_last, f_iter);
      if (!ls_result.converged_) {
        AX_ERROR("Line Search Error: Failed to converge");
      }
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
  sparse_solver_ = std::make_unique<math::SparseSolver_ConjugateGradient>();
  linesearch_ = std::make_unique<Linesearch_Backtracking>();
  math::SparseSolver_ConjugateGradient* cg
      = static_cast<math::SparseSolver_ConjugateGradient*>(sparse_solver_.get());
  cg->SetPreconditioner(std::make_unique<math::Preconditioner_IncompleteCholesky>());
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
