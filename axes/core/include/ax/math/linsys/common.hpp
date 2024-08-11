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
  matxxr A_;
  LinsysProblem_Dense(matxxr const& A) : A_{A} {}
  LinsysProblem_Dense(matxxr&& A) : A_{std::move(A)} {}

  LinsysProblem_Dense() = default;
};

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Dense> make_dense_problem(matxxr const& A) {
  return std::make_unique<LinsysProblem_Dense>(A);
}

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Dense> make_dense_problem(matxxr&& A) {
  return std::make_unique<LinsysProblem_Dense>(std::move(A));
}

struct LinsysProblem_Sparse {
  // Problem Description
  spmatr A_;

  // For Iterative Solvers: Solution Requirement
  real l2_tol_;
  real linf_tol_;

  // Additional checkers.
  std::function<bool(matxxr const&, matxxr const&)> converge_residual_;
  std::function<bool(matxxr const&)> converge_solution_;
  LinsysProblem_Sparse(spmatr const& A) : A_{A} {
    A_.makeCompressed();
  }
  LinsysProblem_Sparse(spmatr&& A) : A_{std::move(A)} {
    A_.makeCompressed();
  }
  LinsysProblem_Sparse() = default;
};

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Sparse> make_sparse_problem(spmatr const& A) {
  return std::make_unique<LinsysProblem_Sparse>(A);
}

AX_FORCE_INLINE std::unique_ptr<LinsysProblem_Sparse> make_sparse_problem(spmatr&& A) {
  return std::make_unique<LinsysProblem_Sparse>(std::move(A));
}

/****************************** Solver Result ******************************/
struct LinsysSolveResult {
  // Basic Result
  matxxr solution_;
  bool converged_;

  // For iterative solver
  idx num_iter_{-1};

  // May be not set?
  real l2_err_{-1};
  real linf_err_{-1};

  LinsysSolveResult(idx rows, idx cols = 1) : solution_(rows, cols), converged_{false} {}
  LinsysSolveResult(matxxr const& solution, bool converged = true)
      : solution_{solution}, converged_{converged} {}
};

}  // namespace ax::math
