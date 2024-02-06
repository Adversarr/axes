#include "axes/optim/newton.hpp"

#include "axes/core/echo.hpp"
#include "axes/math/linalg.hpp"
#include "axes/math/linsys/solver_base.hpp"
#include "axes/optim/linesearch/linesearch.hpp"
#include "axes/math/linsys/dense.hpp"
#include "axes/math/linsys/sparse.hpp"
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
  std::string linsys_name = linsys_opt.Get<std::string>("name", "FullPivLU");
  DLOG(INFO) << "Linsys Solver: " << linsys_name;
  if (is_dense_hessian) {
    auto kind = utils::reflect_enum<math::DenseSolverKind>(linsys_name);
    if (!kind) {
      return utils::FailedPreconditionError("Invalid Dense linsys solver: " + linsys_name);
    }
    dense_solver = math::DenseSolver::Create(kind.value());
    CHECK(dense_solver) << "Invalid Dense linsys solver: " << linsys_name;
  } else {
    // auto kind = utils::reflect_enum<math::SparseSolverKind>(linsys_name);
    // if (!kind) {
    //   return utils::FailedPreconditionError("Invalid Dense linsys solver: " + linsys_name);
    // }
    // sparse_solver = math::SparseSolverBase::Create(kind.value());
    // CHECK(sparse_solver) << "Invalid Sparse linsys solver: " << linsys_name;
  }

  // Initialize
  math::vecxr x = x0;
  real f_iter = problem_.EvalEnergy(x);
  math::vecxr grad = problem_.EvalGrad(x);
  math::vecxr dir = -grad;

  idx iter = 0;
  bool converged = false;
  bool verbose = options.Get<idx>("verbose", problem_.HasVerbose());

  OptResultImpl result;
  bool converge_grad = false;
  bool converge_var = false;
  // Main loop
  for (iter = 0; iter < max_iter_; ++iter) {
    // Verbose
    if (verbose) {
      problem_.EvalVerbose(iter, x, f_iter);
    }

    DLOG(INFO) << "Newton iter " << iter << std::endl
      << "  x: " << x.transpose() << std::endl
      << "  f: " << f_iter << std::endl
      << "  grad: " << grad.transpose();

    // Check convergence
    converge_grad = problem_.HasConvergeGrad() && problem_.EvalConvergeGrad(x, grad) < tol_grad_;
    converge_var = problem_.HasConvergeVar() && problem_.EvalConvergeVar(x, x0) < tol_var_;
    if (converge_grad || converge_var) {
      converged = true;
      break;
    }

    // Find a Dir
    if (is_dense_hessian) {
      math::matxxr H = problem_.EvalHessian(x);
      DLOG(INFO) << std::endl << H;
      math::LinsysProblem_Dense prob(H, -grad, true);
      if (auto status = dense_solver->Analyse(prob, linsys_opt); !status.ok()) {
        return status;
      }
      auto solution = dense_solver->Solve(-grad, {}, linsys_opt);
      if (!solution.ok()) {
        return solution.status();
      }
      dir = solution.value().solution_;
    } else {
      CHECK(false) << "This branch has not been implemented";
    }
    DLOG(INFO) << "Direction: " << dir.transpose();
    // Line search
    auto lsr = line_search->Optimize(problem_, x, dir, ls_opt);
    OptResultImpl ls_result;
    if (!lsr.ok()) {
      LOG(ERROR) << "Line Search Error: " << lsr.status();
      return lsr.status();
    }
    ls_result = std::move(lsr.value());
    DLOG(INFO) << "Step: " << (ls_result.x_opt_ - x).transpose();
    x = ls_result.x_opt_;
    f_iter = ls_result.f_opt_;
    grad = problem_.EvalGrad(x);
  }

  result.converged_grad_ = converge_grad;
  result.converged_var_ = converge_var;
  result.x_opt_ = x;
  result.f_opt_ = f_iter;
  result.n_iter_ = iter;

  DLOG(INFO) << "Newton's Method: " << (converged ? "Converged" : "Not Converged") << std::endl
             << "  Iterations: " << iter << std::endl
             << "  Energy: " << f_iter << std::endl
             << "  Gradient Convergence: " << converge_grad << std::endl
             << "  Variable Convergence: " << converge_var;

  return result;
}

}  // namespace ax::optim
