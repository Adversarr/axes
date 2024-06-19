#include "ax/optim/optimizers/newton.hpp"

#include "ax/core/echo.hpp"
#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/dense/LLT.hpp"
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/utils/status.hpp"
#include "ax/utils/time.hpp"

namespace ax::optim {

OptResult Newton::Optimize(OptProblem const& problem_, math::vecxr const& x0) const {
  AX_TIME_FUNC();
  AX_THROW_IF_FALSE(problem_.HasEnergy(), "Energy function not set");
  AX_THROW_IF_FALSE(problem_.HasGrad(), "Gradient function not set");
  AX_THROW_IF_FALSE(problem_.HasHessian() || problem_.HasSparseHessian(), "Hessian function not set");
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
      AX_LOG(INFO) << "Newton iter " << iter << std::endl
                    << "  x: " << x.transpose() << std::endl
                    << "  f: " << f_iter << std::endl
                    << "  grad: " << grad.transpose();
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
      AX_THROW_IF_TRUE(H.rows() != x.rows() || H.cols() != x.rows(), "Hessian matrix size mismatch");
      math::LinsysProblem_Dense prob(std::move(H), grad);
      auto solution = dense_solver_->SolveProblem(prob);
      dir = -solution.solution_;
    } else {
      AX_TIMEIT("Eval and Solve Sparse System");
      math::sp_matxxr H = problem_.EvalSparseHessian(x);
      AX_THROW_IF_TRUE(H.rows() != x.rows() || H.cols() != x.rows(), "Hessian matrix size mismatch");
      math::LinsysProblem_Sparse prob(H, grad);
      if (problem_.HasConvergeGrad()) {
        prob.converge_residual_ = [&](math::vecxr const& x, math::vecxr const& r) -> bool {
          return problem_.EvalConvergeGrad(x, r) < tol_grad_;
        };
      }
      if (problem_.HasConvergeVar()) {
        prob.converge_solution_ = [&](math::vecxr const& x) -> bool {
          return problem_.EvalConvergeVar(x, x0) < tol_var_;
        };
      }
      auto solution = sparse_solver_->SolveProblem(prob);
      dir = -solution.solution_;
    }

    real dir_dot_grad = dir.dot(grad);
    if (dir_dot_grad > 0) {
      AX_LOG(ERROR) << "Direction is not descent!!!";
      break;
    }

    // SECT: Line search
    OptResult ls_result = linesearch_->Optimize(problem_, x, grad, dir);
    x = ls_result.x_opt_;
    real f_last = f_iter;
    f_iter = ls_result.f_opt_;
    if (f_iter > f_last) {
      AX_LOG(ERROR) << "Line Search Error: Energy increased!!!";
      if (!ls_result.converged_) {
        AX_LOG(ERROR) << "Line Search Error: Failed to converge!!!";
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

Newton::Newton() {
  dense_solver_ = std::make_unique<math::DenseSolver_LLT>();
  sparse_solver_ = std::make_unique<math::SparseSolver_ConjugateGradient>();
  linesearch_ = std::make_unique<Linesearch_Backtracking>();
  linesearch_name_ = "kBacktracking";
  dense_solver_name_ = "kLDLT";
  sparse_solver_name_ = "kConjugateGradient";
  math::SparseSolver_ConjugateGradient *cg = dynamic_cast<math::SparseSolver_ConjugateGradient *>(sparse_solver_.get());
  cg->SetPreconditioner(std::make_unique<math::Preconditioner_IncompleteCholesky>());
}

void Newton::SetOptions(utils::Opt const& options) {
  OptimizerBase::SetOptions(options);
  AX_SYNC_OPT_IF(options, std::string, linesearch_name) {
    auto ls = utils::reflect_enum<LineSearchKind>(linesearch_name_);
    AX_CHECK(ls) << "Unknown linesearch_name: " << linesearch_name_;
    linesearch_ = LinesearchBase::Create(ls.value());
    utils::sync_from_opt(*linesearch_, options, "linesearch_opt");
  }

  AX_SYNC_OPT_IF(options, std::string, dense_solver_name) {
    auto ds = utils::reflect_enum<math::DenseSolverKind>(dense_solver_name_);
    AX_CHECK(ds) << "Unknown dense_solver_name: " << dense_solver_name_;
    dense_solver_ = math::DenseSolverBase::Create(ds.value());
    utils::sync_from_opt(*dense_solver_, options, "dense_solver_opt");
  }

  AX_SYNC_OPT_IF(options, std::string, sparse_solver_name) {
    auto ss = utils::reflect_enum<math::SparseSolverKind>(sparse_solver_name_);
    AX_CHECK(ss) << "Unknown sparse_solver_name: " << sparse_solver_name_;
    sparse_solver_ = math::SparseSolverBase::Create(ss.value());
    utils::sync_from_opt(*sparse_solver_, options, "sparse_solver_opt");
  }
}

utils::Opt Newton::GetOptions() const {
  utils::Opt opt = OptimizerBase::GetOptions();
  opt["linesearch_name"] = linesearch_name_;
  opt["dense_solver_name"] = dense_solver_name_;
  opt["sparse_solver_name"] = sparse_solver_name_;
  opt["linesearch_opt"] = linesearch_->GetOptions();
  opt["dense_solver_opt"] = dense_solver_->GetOptions();
  opt["sparse_solver_opt"] = sparse_solver_->GetOptions();
  return opt;
}

}  // namespace ax::optim
