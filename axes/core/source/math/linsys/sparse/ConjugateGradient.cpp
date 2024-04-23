#include "ax/math/linsys/sparse/ConjugateGradient.hpp"
#include "ax/math/linalg.hpp"

namespace ax::math {

Status SparseSolver_ConjugateGradient::Analyse(LinsysProblem_Sparse const &problem) {
  if (preconditioner_) {
    sparse_problem_ = problem;
    AX_RETURN_NOTOK(preconditioner_->Analyse(sparse_problem_));
  } else {
    solver_.compute(problem.A_);
    if (solver_.info() != Eigen::Success) {
      return utils::FailedPreconditionError("SparseSolver_ConjugateGradient: factorization failed");
    }
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_ConjugateGradient::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // ERROR: Use the Eigen Solver!!!
    vecxr x;
    if (x0.size() > 0) {
      if (x0.size() != b.size()) {
        return utils::FailedPreconditionError("Size mismatch!");
      }
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResultImpl impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    sp_matxxr const& A = sparse_problem_.A_;
    // Initialize the solution vector.
    math::vecxr x = x0;
    if (x.size() != A.cols()) {
      x.resize(A.cols());
      x.setZero();
    }

    // Initialize the residual vector.
    math::vecxr r = A * x - b;
    math::vecxr y = preconditioner_->Solve(r, x0);
    math::vecxr p = -y;
    math::vecxr Ap(p.size());
    bool converged = false, conv_residual = false;
    idx iter = 0;
    for (; iter < max_iter_; ++iter) {
      Ap.noalias() = A * p;
      real alpha = r.dot(y) / p.dot(Ap);
      x += alpha * p;  // 5.39b
      r += alpha * Ap; // 5.39c
      real residual_norm = math::norm(r);
      if (residual_norm <= tol_) {
        converged = true;
        break;
      } else if (sparse_problem_.converge_residual_ && sparse_problem_.converge_residual_(x, r)) {
        conv_residual = true;
        break;
      } else if (sparse_problem_.converge_solution_ && sparse_problem_.converge_solution_(x)) {
        converged = true;
        break;
      }
      y.noalias() = preconditioner_->Solve(r, x0);
      real beta = y.dot(r) / y.dot(r);
      p = -y + beta * p;
    }
    LinsysSolveResultImpl impl(x, converged || conv_residual);
    impl.num_iter_ = iter;
    return impl;
  }
}

Status SparseSolver_ConjugateGradient::SetOptions(utils::Opt const &opt) {
  AX_SYNC_OPT_IF(opt, idx, max_iter){
    if (max_iter_ < 1) {
      return utils::InvalidArgumentError("max_iter must be positive");
    }

    solver_.setMaxIterations(max_iter_);
  }

  AX_SYNC_OPT_IF(opt, real, tol){
    if (tol_ < 0) {
      return utils::InvalidArgumentError("tol must be non-negative");
    }
    solver_.setTolerance(tol_);
  }

  if (opt.Holds<std::string>("preconditioner")) {
    std::string precond = opt.Get<std::string>("preconditioner");
    auto kind = utils::reflect_enum<PreconditionerKind>(precond);
    if (!kind) {
      return utils::InvalidArgumentError("Invalid preconditioner kind: " + precond);
    }
    auto pc = PreconditionerBase::Create(kind.value());
    if (!pc) {
      return utils::InvalidArgumentError("Failed to create preconditioner: " + precond);
    }
    pc.swap(preconditioner_);
  }

  if (opt.Holds<utils::Opt>("preconditioner_options")) {
    return preconditioner_->SetOptions(opt.Get<utils::Opt>("preconditioner_options"));
  }

  AX_RETURN_OK();
}

utils::Opt SparseSolver_ConjugateGradient::GetOptions() const {
  utils::Opt opt {
    {"max_iter", max_iter_},
    {"tol", tol_},
  };
  if (preconditioner_) {
    auto name = utils::reflect_name(preconditioner_->Kind());
    if (!name) {
      AX_LOG(FATAL) << "Invalid preconditioner kind: " << static_cast<idx>(preconditioner_->Kind());
    }
    opt.Emplace("preconditioner", name.value());
    opt.Emplace("preconditioner_options", preconditioner_->GetOptions());
  }
  return opt;
}

}  // namespace ax::math
