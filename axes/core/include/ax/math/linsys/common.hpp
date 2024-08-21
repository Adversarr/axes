#pragma once

#include "ax/math/common.hpp"
#include "ax/math/sparse.hpp"

namespace ax::math {

/****************************** Problem Definition ******************************/

// NOTE: There are 3 kinds of Linear Systems:
//  1. Dense Matrix.
//  2. Sparse Matrix
//  3. Implicit

struct LinsysProblem_Dense {
  // Problem Description
  RealMatrixX A_;
  LinsysProblem_Dense(RealMatrixX const& A) : A_{A} {}
  LinsysProblem_Dense(RealMatrixX&& A) : A_{std::move(A)} {}

  LinsysProblem_Dense() = default;
};

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Dense> make_dense_problem(RealMatrixX const& A) {
  return std::make_unique<LinsysProblem_Dense>(A);
}

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Dense> make_dense_problem(RealMatrixX&& A) {
  return std::make_unique<LinsysProblem_Dense>(std::move(A));
}

struct LinsysProblem_Sparse {
  // Problem Description
  RealSparseMatrix A_;

  // For Iterative Solvers: Solution Requirement
  Real l2_tol_;
  Real linf_tol_;

  // Additional checkers.
  std::function<bool(RealMatrixX const&, RealMatrixX const&)> converge_residual_;
  std::function<bool(RealMatrixX const&)> converge_solution_;
  LinsysProblem_Sparse(RealSparseMatrix const& A) : A_{A} {
    A_.makeCompressed();
  }
  LinsysProblem_Sparse(RealSparseMatrix&& A) : A_{std::move(A)} {
    A_.makeCompressed();
  }
  LinsysProblem_Sparse() = default;
};

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Sparse> make_sparse_problem(RealSparseMatrix const& A) {
  return std::make_unique<LinsysProblem_Sparse>(A);
}

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Sparse> make_sparse_problem(RealSparseMatrix&& A) {
  return std::make_unique<LinsysProblem_Sparse>(std::move(A));
}

/****************************** Solver Result ******************************/
struct LinsysSolveResult {
  // Basic Result
  RealMatrixX solution_;
  bool converged_;

  // For iterative solver
  Index num_iter_{-1};

  // May be not set?
  Real l2_err_{-1};
  Real linf_err_{-1};

  LinsysSolveResult(Index rows, Index cols = 1) : solution_(rows, cols), converged_{false} {}
  LinsysSolveResult(RealMatrixX const& solution, bool converged = true)
      : solution_{solution}, converged_{converged} {}
};

}  // namespace ax::math
