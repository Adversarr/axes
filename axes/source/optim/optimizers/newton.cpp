#include "axes/optim/optimizers/newton.hpp"

#include "axes/core/echo.hpp"
#include "axes/math/linsys/dense.hpp"
#include "axes/math/linsys/sparse.hpp"
#include "axes/optim/linesearch/linesearch.hpp"
#include "axes/utils/status.hpp"

namespace ax::optim {

OptResult Newton::Optimize(math::vecxr const& x0, utils::Opt const& options) {
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

  // SECT: Setup Linesearch
  utils::uptr<LineSearchBase> line_search;
  utils::Opt ls_opt;
  std::string ls_name;
  if (options.Has<utils::Opt>("line_search")) {
    ls_opt = options.Get<utils::Opt>("line_search");
    ls_name = ls_opt.Get<std::string>("name", "backtracking");
  }
  DLOG(INFO) << "Linesearch Method: " << ls_name;
  auto ls_kind_opt = utils::reflect_enum<LineSearchKind>(ls_name);
  auto ls_kind = ls_kind_opt.value_or(LineSearchKind::kBacktracking);
  line_search = LineSearchBase::Create(ls_kind);
  if (!line_search) {
    return utils::FailedPreconditionError("Invalid line search method: " + ls_name);
  }

  // SECT: Setup Linsys Solver
  utils::Opt linsys_opt = options.Get<utils::Opt>("linsys", utils::Opt{});
  utils::uptr<math::DenseSolver> dense_solver;
  utils::uptr<math::SparseSolverBase> sparse_solver;
  if (is_dense_hessian) {
    std::string linsys_name = linsys_opt.Get<std::string>("name", "LDLT");
    DLOG(INFO) << "Dense Linsys Solver: " << linsys_name;
    auto kind = utils::reflect_enum<math::DenseSolverKind>(linsys_name);
    if (!kind) {
      return utils::FailedPreconditionError("Invalid Dense linsys solver: " + linsys_name);
    }
    dense_solver = math::DenseSolver::Create(kind.value());
    CHECK(dense_solver) << "Invalid Dense linsys solver: " << linsys_name;
  } else {
    std::string linsys_name = linsys_opt.Get<std::string>("name", "LDLT");
    DLOG(INFO) << "Sparse Linsys Solver: " << linsys_name;
    auto kind = utils::reflect_enum<math::SparseSolverKind>(linsys_name);
    if (!kind) {
      return utils::FailedPreconditionError("Invalid Sparse linsys solver: " + linsys_name);
    }
    sparse_solver = math::SparseSolverBase::Create(kind.value());
    CHECK(sparse_solver) << "Invalid Sparse linsys solver: " << linsys_name;
  }

  // SECT: Initialize
  math::vecxr x = x0;
  real f_iter = problem_.EvalEnergy(x);
  math::vecxr grad = problem_.EvalGrad(x);
  math::vecxr dir = -grad;

  idx iter = 0;
  bool converged = false;
  bool verbose = options.Get<idx>("verbose", problem_.HasVerbose() ? 1 : 0);

  OptResultImpl result;
  bool converge_grad = false;
  bool converge_var = false;
  // Main loop
  for (iter = 0; iter < max_iter_; ++iter) {
    // SECT: Verbose
    if (verbose) {
      problem_.EvalVerbose(iter, x, f_iter);
      DLOG(INFO) << "Newton iter " << iter << std::endl
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
      math::LinsysProblem_Dense prob(H, -grad, true);
      auto solution = dense_solver->Solve(prob, linsys_opt);
      if (!solution.ok()) {
        LOG(ERROR) << "Dense Linsys Solver Error: Hessian may not be invertible";
        return solution.status();
      }
      dir = std::move(solution.value().solution_);
    } else {
      math::sp_matxxr H = problem_.EvalSparseHessian(x);
      math::LinsysProblem_Sparse prob(H, -grad, true);
      auto solution = sparse_solver->Solve(prob, linsys_opt);
      if (!solution.ok()) {
        LOG(ERROR) << "Sparse Linsys Solver Error: Hessian may not be invertible";
        return solution.status();
      }
      LOG(ERROR) << "Solution: " << solution.value().solution_.transpose();
    }

    // SECT: Line search
    auto lsr = line_search->Optimize(problem_, x, dir, ls_opt);
    OptResultImpl ls_result;
    if (!lsr.ok()) {
      LOG(ERROR) << "Line Search Error: " << lsr.status();
      LOG(ERROR) << "Search Dir = " << dir.transpose();
      LOG(ERROR) << "Possible Reason: Your Hessian matrix is not SPSD";
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

}  // namespace ax::optim
