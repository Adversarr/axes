#include "axes/math/linsys/sparse.hpp"

#include "axes/utils/status.hpp"

namespace ax::math {

utils::uptr<SparseSolverBase> SparseSolverBase::Create(SparseSolverKind kind) {
  switch (kind) {
    case kLDLT:
      return std::make_unique<SparseSolver_LDLT>();
    case kLLT:
      return std::make_unique<SparseSolver_LLT>();
    case kLU:
      return std::make_unique<SparseSolver_LU>();
    case kQR:
      return std::make_unique<SparseSolver_QR>();
    case kConjugateGradient:
      return std::make_unique<SparseSolver_ConjugateGradient>();
    case kLeastSquaresConjugateGradient:
      return std::make_unique<SparseSolver_LeastSquaresConjugateGradient>();
    case kBiCGSTAB:
      return std::make_unique<SparseSolver_BiCGSTAB>();
    default:
      return nullptr;
  }
}

Status SparseSolver_LDLT::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);

  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_LDLT: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_LDLT::Solve(vecxr const &b, vecxr const &x0) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_LDLT: solve failed");
  }
  return x;
}

Status SparseSolver_LLT::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_LLT: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_LLT::Solve(vecxr const &b, vecxr const &x0) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_LLT: solve failed");
  }
  return x;
}

Status SparseSolver_LU::Analyse(LinsysProblem_Sparse const &problem) {
  if (!problem.A_.isCompressed()) {
    return utils::InvalidArgumentError("SparseSolver_LU: matrix is not compressed");
  }
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_LU: analyzePattern failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_LU::Solve(vecxr const &b, vecxr const &x0) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_LU: solve failed");
  }
  return x;
}

Status SparseSolver_QR::Analyse(LinsysProblem_Sparse const &problem) {
  if (!problem.A_.isCompressed()) {
    return utils::InvalidArgumentError("SparseSolver_CG: matrix is not compressed");
  }
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_CG: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_QR::Solve(vecxr const &b, vecxr const &x0) {
  vecxr x = solver_.solve(b);
  if (solver_.info() != Eigen::Success) {
    return utils::InvalidArgumentError("SparseSolver_CG: solve failed");
  }
  return x;
}

Status SparseSolver_ConjugateGradient::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_ConjugateGradient: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_ConjugateGradient::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // TODO: Fix bug.
    // solver_.setMaxIterations(options.GetDefault<idx>("max_iter", 100));
    vecxr x;
    if (x0.size() > 0) {
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResultImpl impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    CHECK(false) << "This branch have not been implemented yet";
  }
}

Status SparseSolver_LeastSquaresConjugateGradient::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError(
        "SparseSolver_LeastSquaresConjugateGradient: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_LeastSquaresConjugateGradient::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // TODO: Fix bug
    // solver_.setMaxIterations(options.GetDefault<idx>("max_iter", 100));
    vecxr x;
    if (x0.size() > 0) {
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResultImpl impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    CHECK(false) << "This branch have not been implemented yet";
  }
}

Status SparseSolver_BiCGSTAB::Analyse(LinsysProblem_Sparse const &problem) {
  solver_.compute(problem.A_);
  if (solver_.info() != Eigen::Success) {
    return utils::FailedPreconditionError("SparseSolver_BiCGSTAB: factorization failed");
  }

  AX_RETURN_OK();
}

LinsysSolveResult SparseSolver_BiCGSTAB::Solve(vecxr const &b, vecxr const &x0) {
  if (!preconditioner_) {
    // TODO: Fix bug
    // solver_.setMaxIterations(options.GetDefault<idx>("max_iter", 100));
    vecxr x;
    if (x0.size() > 0) {
      x = solver_.solveWithGuess(b, x0);
    } else {
      x = solver_.solve(b);
    }
    LinsysSolveResultImpl impl(x, solver_.info() == Eigen::Success);
    impl.num_iter_ = solver_.iterations();
    impl.l2_err_ = solver_.error();
    return impl;
  } else {
    CHECK(false) << "This branch have not been implemented yet";
  }
}

}  // namespace ax::math
