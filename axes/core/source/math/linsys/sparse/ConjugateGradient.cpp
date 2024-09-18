#include "ax/math/linsys/sparse/ConjugateGradient.hpp"

#include "ax/core/excepts.hpp"
#include "ax/math/linalg.hpp"
#include "ax/math/linsys/common.hpp"
#include "ax/math/linsys/sparse.hpp"


namespace ax::math {

void SparseSolver_ConjugateGradient::AnalyzePattern() {
  if (preconditioner_) {
    preconditioner_->SetProblem(cached_problem_);
    preconditioner_->AnalyzePattern();
  } else {
    solver_.analyzePattern(cached_problem_->A_);
    AX_THROW_IF_FALSE(solver_.info() == Eigen::Success,
                      "SparseSolver_ConjugateGradient: factorization failed");
  }
}

void SparseSolver_ConjugateGradient::Factorize() {
  if (preconditioner_) {
    preconditioner_->Factorize();
  } else {
    solver_.factorize(cached_problem_->A_);
    AX_THROW_IF_FALSE(solver_.info() == Eigen::Success,
                      "SparseSolver_ConjugateGradient: factorization failed");
  }
}

static void batch_dot_to(RealMatrixX const &lhs, RealMatrixX const &rhs, RealVectorX &dest) {
  for (Index i = 0; i < dest.rows(); ++i) {
    dest[i] = dot(lhs.col(i), rhs.col(i));
  }
}

LinsysSolveResult SparseSolver_ConjugateGradient::Solve(RealMatrixX const &b, RealMatrixX const &x0) {
  RealSparseMatrix const &A = cached_problem_->A_;
  AX_THROW_IF_NE(b.rows(), A.rows(), "Invalid rhs vector size: {} != {} (b, A)", b.rows(),
                 A.rows());
  if (!preconditioner_) {
    RealMatrixX x;
    if (x0.size() > 0) {
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
    // Initialize the solution vector.
    RealMatrixX x = x0;
    if (x.rows() != A.cols()) {
      AX_THROW_IF_NE(x.rows(), 0, "Invalid initial guess size");
      x.resize(A.cols(), b.cols());
      x.setZero();
    }

    // Initialize the residual vector.
    RealMatrixX r = A * x - b;
    RealMatrixX y = preconditioner_->Solve(r);
    RealMatrixX p = -y;
    RealMatrixX p_new = p;
    RealMatrixX Ap = p;

    Index const ncols = x.cols();
    RealVectorX rk_dot_yk(ncols), rk_dot_yk_new(ncols), p_dot_Ap(ncols);
    bool converged = false, conv_residual = false;
    Index iter = 0;
    batch_dot_to(r, y, rk_dot_yk);
    for (; iter < max_iter_; ++iter) {
      Ap.noalias() = A * p;
      // Real alpha = r.dot(y) / p.dot(Ap); // BUG: alpha is row vector.
      // x.noalias() += alpha * p;   // 5.39b
      // r.noalias() += alpha * Ap;  // 5.39c
      batch_dot_to(p, Ap, p_dot_Ap);
      for (Index i = 0; i < ncols; ++i) {
        Real const alpha = rk_dot_yk[i] / p_dot_Ap[i];
        x.col(i).noalias() += alpha * p.col(i);
        r.col(i).noalias() += alpha * Ap.col(i);
      }

      Real residual_norm = norm(r);
      if (residual_norm <= tol_) {
        converged = true;
        break;
      } else if (cached_problem_->converge_residual_ && cached_problem_->converge_residual_(x, r)) {
        conv_residual = true;
        break;
      } else if (cached_problem_->converge_solution_ && cached_problem_->converge_solution_(x)) {
        converged = true;
        break;
      }

      y.noalias() = preconditioner_->Solve(r);
      batch_dot_to(r, y, rk_dot_yk_new);
      // Real beta = rk_dot_yk_new / rk_dot_yk;
      // rk_dot_yk = rk_dot_yk_new;
      // p_new.noalias() = beta * p - y;
      for (Index i = 0; i < ncols; ++i) {
        Real const beta = rk_dot_yk_new[i] / rk_dot_yk[i];
        p_new.col(i).noalias() = beta * p.col(i);
      }
      p_new.noalias() -= y;
      rk_dot_yk.swap(rk_dot_yk_new);
      p_new.swap(p);
    }
    LinsysSolveResult impl(x, converged || conv_residual);
    impl.num_iter_ = iter;
    return impl;
  }
}

void SparseSolver_ConjugateGradient::SetOptions(utils::Options const &opt) {
  AX_SYNC_OPT_IF(opt, Index, max_iter) {
    // if (max_iter_ < 1) {
    //   return utils::InvalidArgumentError("max_iter must be positive");
    // }
    AX_THROW_IF_LT(max_iter_, 1, "max_iter must be positive");

    solver_.setMaxIterations(max_iter_);
  }

  AX_SYNC_OPT_IF(opt, Real, tol) {
    AX_THROW_IF_LT(tol_, 0, "tol must be non-negative");
    solver_.setTolerance(tol_);
  }

  PreconditionerKind pk;
  if (auto it = opt.find("preconditioner"); it != opt.end()) {
    if (it->value().is_string()) {
      std::string name = it->value().as_string().c_str();
      auto kind = utils::reflect_enum<PreconditionerKind>(name);
      AX_THROW_IF_NULL(kind, "Invalid preconditioner kind: {}", name);
      pk = kind.value();
      preconditioner_ = PreconditionerBase::Create(pk);
      AX_THROW_IF_NULLPTR(preconditioner_, "Failed to create preconditioner: {}", name);
    } else {
      AX_THROW_RUNTIME_ERROR("Expect value under 'preconditioner' to be a string.");
    }
  }

  if (auto it = opt.find("preconditioner_opt"); it != opt.end()) {
    if (preconditioner_ || it->value().is_object()) {
      preconditioner_->SetOptions(it->value().as_object());
    }
  }
  SparseSolverBase::SetOptions(opt);
}

utils::Options SparseSolver_ConjugateGradient::GetOptions() const {
  utils::Options opt{
      {"max_iter", max_iter_},
      {"tol", tol_},
  };
  if (preconditioner_) {
    auto name = utils::reflect_name(preconditioner_->GetKind());
    if (!name) {
      AX_THROW_RUNTIME_ERROR("Invalid preconditioner kind: {}",
                               static_cast<Index>(preconditioner_->GetKind()));
    }
    opt.insert_or_assign("preconditioner", name.value());
    opt.insert_or_assign("preconditioner_opt", preconditioner_->GetOptions());
  }
  return opt;
}

}  // namespace ax::math
