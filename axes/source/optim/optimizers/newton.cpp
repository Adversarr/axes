#include "axes/optim/optimizers/newton.hpp"

#include "axes/core/echo.hpp"
#include "axes/math/linsys/dense.hpp"
#include "axes/math/linsys/dense/LDLT.hpp"
#include "axes/math/linsys/sparse.hpp"
#include "axes/math/linsys/sparse/LDLT.hpp"
#include "axes/optim/linesearch/backtracking.hpp"
#include "axes/optim/linesearch/linesearch.hpp"
#include "axes/utils/status.hpp"

namespace ax::optim {

OptResult Newton::Optimize(OptProblem const& problem_, math::vecxr const& x0) const {
  if (!problem_.HasEnergy()) {
    return utils::FailedPreconditionError("Energy function not set");
  }
  if (!problem_.HasGrad()) {
    return utils::FailedPreconditionError("Gradient function not set");
  }

  if (!problem_.HasHessian() && !problem_.HasSparseHessian()) {
    return utils::FailedPreconditionError("Hessian function not set");
  }
  bool is_dense_hessian = problem_.HasHessian();

  // SECT: Initialize
  math::vecxr x = x0;
  real f_iter = problem_.EvalEnergy(x);
  math::vecxr grad = problem_.EvalGrad(x);
  math::vecxr dir = -grad;

  idx iter = 0;
  bool converged = false;

  OptResultImpl result;
  bool converge_grad = false;
  bool converge_var = false;
  // Main loop
  for (iter = 0; iter < max_iter_; ++iter) {
    // SECT: verbose_
    if (verbose_) {
      problem_.EvalVerbose(iter, x, f_iter);
      AX_DLOG(INFO) << "Newton iter " << iter << std::endl
                    << "  x: " << x.transpose() << std::endl
                    << "  f: " << f_iter << std::endl
                    << "  grad: " << grad.transpose();
    }

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
      if (H.rows() != x.rows() || H.cols() != x.rows()) {
        return utils::FailedPreconditionError("Hessian matrix size mismatch");
      }
      math::LinsysProblem_Dense prob(std::move(H), grad);
      auto solution = dense_solver_->SolveProblem(prob);
      if (!solution.ok()) {
        AX_LOG(ERROR) << "Dense Linsys Solver Error: Hessian may not be invertible";
        return solution.status();
      }
      dir = -solution.value().solution_;
    } else {
      math::sp_matxxr H = problem_.EvalSparseHessian(x);
      if (H.rows() != x.rows() || H.cols() != x.rows()) {
        return utils::FailedPreconditionError("Hessian matrix size mismatch");
      }
      math::LinsysProblem_Sparse prob(std::move(H), grad);
      auto solution = sparse_solver_->SolveProblem(prob);
      if (!solution.ok()) {
        AX_LOG(ERROR) << "Sparse Linsys Solver Error: Hessian may not be invertible";
        return solution.status();
      }
      dir = -solution.value().solution_;
    }

    // SECT: Line search
    auto lsr = linesearch_->Optimize(problem_, x, dir);
    OptResultImpl ls_result;
    if (!lsr.ok()) {
      AX_LOG(ERROR) << "Line Search Error: " << lsr.status()
                    << "Possible Reason: Your Hessian matrix is not SPSD";
      AX_LOG(ERROR) << "Search Dir = " << dir.transpose();
      return lsr.status();
    }
    ls_result = std::move(lsr.value());
    x = ls_result.x_opt_;
    f_iter = ls_result.f_opt_;
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
  dense_solver_ = std::make_unique<math::DenseSolver_LDLT>();
  sparse_solver_ = std::make_unique<math::SparseSolver_LDLT>();
  linesearch_ = std::make_unique<BacktrackingLinesearch>();
  BacktrackingLinesearch* ls = dynamic_cast<BacktrackingLinesearch*>(linesearch_.get());
  ls->alpha_ = 1.0;
  ls->c_ = 0.1;
  ls->rho_ = 0.5;
  linesearch_name_ = "Backtracking";
  dense_solver_name_ = "LDLT";
  sparse_solver_name_ = "LDLT";
}

Status Newton::SetOptions(utils::Opt const& options) {
  AX_SYNC_OPT_IF(options, std::string, linesearch_name) {
    auto ls = utils::reflect_enum<LineSearchKind>(linesearch_name_);
    AX_CHECK(ls) << "Unknown linesearch_name: " << linesearch_name_;
    linesearch_ = LinesearchBase::Create(ls.value());
    AX_RETURN_NOTOK_OR(utils::sync_to_field(*linesearch_, options, "linesearch_opt"));
  }

  AX_SYNC_OPT_IF(options, std::string, dense_solver_name) {
    auto ds = utils::reflect_enum<math::DenseSolverKind>(dense_solver_name_);
    AX_CHECK(ds) << "Unknown dense_solver_name: " << dense_solver_name_;
    dense_solver_ = math::DenseSolverBase::Create(ds.value());
    AX_RETURN_NOTOK_OR(utils::sync_to_field(*dense_solver_, options, "dense_solver_opt"));
  }

  AX_SYNC_OPT_IF(options, std::string, sparse_solver_name) {
    auto ss = utils::reflect_enum<math::SparseSolverKind>(sparse_solver_name_);
    AX_CHECK(ss) << "Unknown sparse_solver_name: " << sparse_solver_name_;
    sparse_solver_ = math::SparseSolverBase::Create(ss.value());
    AX_RETURN_NOTOK_OR(utils::sync_to_field(*sparse_solver_, options, "sparse_solver_opt"));
  }
  return OptimizerBase::SetOptions(options);
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
