#include "ax/math/linsys/sparse/ConjugateGradient.hpp"

#include "ax/core/excepts.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/linsys/common.hpp"
#include "ax/math/linsys/solver_base.hpp"
#include "ax/math/linsys/sparse.hpp"

namespace ax::math {

void SparseSolver_ConjugateGradient::Analyse(LinsysProblem_Sparse const &problem) {
  if (preconditioner_) {
    sparse_problem_ = problem;
    // AX_RETURN_NOTOK(preconditioner_->Analyse(sparse_problem_));
    preconditioner_->Analyse(sparse_problem_);
  } else {
    solver_.compute(problem.A_);
    // if (solver_.info() != Eigen::Success) {
    //   return utils::FailedPreconditionError("SparseSolver_ConjugateGradient: factorization failed");
    // }
    AX_THROW_IF_FALSE(solver_.info() == Eigen::Success, "SparseSolver_ConjugateGradient: factorization failed");
  }
}

LinsysSolveResult SparseSolver_ConjugateGradient::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // ERROR: Use the Eigen Solver!!!
    vecxr x;
    if (x0.size() > 0) {
      // if (x0.size() != b.size()) {
      //   return utils::FailedPreconditionError("Size mismatch!");
      // }
      AX_THROW_IF_NE(x0.size(), b.size(), "Size mismatch!");
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResult impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    sp_matxxr const &A = sparse_problem_.A_;
    // if (b.size() != A.rows()) {
    //   return utils::InvalidArgumentError("Invalid rhs vector: b" + std::to_string(b.size())
    //                                      + " != A" + std::to_string(A.rows()));
    // }
    AX_THROW_IF_NE(b.size(), A.rows(), "Invalid rhs vector: b" + std::to_string(b.size())
                                       + " != A" + std::to_string(A.rows()));
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
    math::vecxr p_new(r.size());
    math::vecxr Ap(p.size());
    real rk_dot_yk = r.dot(y);
    bool converged = false, conv_residual = false;
    idx iter = 0;
    for (; iter < max_iter_; ++iter) {
      Ap.noalias() = A * p;
      real alpha = r.dot(y) / p.dot(Ap);
      x.noalias() += alpha * p;   // 5.39b
      r.noalias() += alpha * Ap;  // 5.39c
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
      real rk_dot_yk_new = r.dot(y);
      real beta = rk_dot_yk_new / rk_dot_yk;
      rk_dot_yk = rk_dot_yk_new;
      p_new.noalias() = beta * p - y;
      p_new.swap(p);
    }
    LinsysSolveResult impl(x, converged || conv_residual);
    impl.num_iter_ = iter;
    return impl;
  }
}

void SparseSolver_ConjugateGradient::SetOptions(utils::Opt const &opt) {
  AX_SYNC_OPT_IF(opt, idx, max_iter) {
    // if (max_iter_ < 1) {
    //   return utils::InvalidArgumentError("max_iter must be positive");
    // }
    AX_THROW_IF_LT(max_iter_, 1, "max_iter must be positive");

    solver_.setMaxIterations(max_iter_);
  }

  AX_SYNC_OPT_IF(opt, real, tol) {
    // if (tol_ < 0) {
    //   return utils::InvalidArgumentError("tol must be non-negative");
    // }
    AX_THROW_IF_LT(tol_, 0, "tol must be non-negative");
    solver_.setTolerance(tol_);
  }

  PreconditionerKind pk;
  if (auto it = opt.find("preconditioner"); it != opt.end()) {
    if (it->value().is_string()) {
      std::string name = it->value().as_string().c_str();
      auto kind = utils::reflect_enum<PreconditionerKind>(name);
      // if (!kind) {
      //   return utils::InvalidArgumentError("Invalid preconditioner kind: "
      //                                      + std::string(it->value().as_string().c_str()));
      // }
      AX_THROW_IF_NULL(kind, "Invalid preconditioner kind: " + name);
      pk = kind.value();
      preconditioner_ = PreconditionerBase::Create(pk);
      // if (!preconditioner_) {
      //   return utils::FailedPreconditionError("Failed to create preconditioner: " + name);
      // }
      AX_THROW_IF_NULLPTR(preconditioner_, "Failed to create preconditioner: " + name);
    } else {
      // return utils::InvalidArgumentError("Expect value under 'preconditioner' to be a string.");
      throw RuntimeError("Expect value under 'preconditioner' to be a string.");
    }
  }

  if (auto it = opt.find("preconditioner_options"); it != opt.end()) {
    if (preconditioner_ || it->value().is_object()) {
      preconditioner_->SetOptions(it->value().as_object());
    }
  }
  SparseSolverBase::SetOptions(opt);
}

utils::Opt SparseSolver_ConjugateGradient::GetOptions() const {
  utils::Opt opt{
      {"max_iter", max_iter_},
      {"tol", tol_},
  };
  if (preconditioner_) {
    auto name = utils::reflect_name(preconditioner_->Kind());
    if (!name) {
      AX_LOG(FATAL) << "Invalid preconditioner kind: " << static_cast<idx>(preconditioner_->Kind());
    }
    opt.insert_or_assign("preconditioner", name.value());
    opt.insert_or_assign("preconditioner_options", preconditioner_->GetOptions());
  }
  return opt;
}

}  // namespace ax::math
