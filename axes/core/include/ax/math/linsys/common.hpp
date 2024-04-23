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
  vecxr b_;

  LinsysProblem_Dense(matxxr A, vecxr b) : A_{std::move(A)}, b_{std::move(b)} {}

  LinsysProblem_Dense() = default;
};

struct LinsysProblem_Sparse {
  // Problem Description
  sp_matxxr A_;
  vecxr b_;

  // For Iterative Solvers: Solution Requirement
  real l2_tol_;
  real linf_tol_;

  // Additional checkers.
  std::function<bool(vecxr const&, vecxr const&)> converge_residual_;
  std::function<bool(vecxr const&)> converge_solution_;
  LinsysProblem_Sparse(sp_matxxr const& A, vecxr const& b) : A_{A}, b_{b} {}

  LinsysProblem_Sparse() = default;
};

struct LinsysProblem_Implicit {
  // Problem Description
  std::function<vecxr(vecxr const&)> A_;
  std::function<vecxr(vecxr const&)> At_;
  std::function<vecxr(vecxr const&)> AtA_;
  vecxr const& b_;
  bool is_spsd_;

  // For Iterative Solvers: Solution Requirement
  real l2_tol_;
  real linf_tol_;

  // Additional checkers.
  std::function<bool(vecxr const&)> converge_residual_;
  std::function<bool(vecxr const&)> converge_solution_;
};

/****************************** Solver Result ******************************/
struct LinsysSolveResultImpl {
  // Basic Result
  vecxr solution_;
  bool converged_;

  // For iterative solver
  idx num_iter_;

  // May be not set?
  real l2_err_{-1};
  real linf_err_{-1};

  LinsysSolveResultImpl(vecxr solution, bool converged, idx num_iter, real l2_err, real linf_err)
      : solution_{std::move(solution)},
        converged_(converged),
        num_iter_{num_iter},
        l2_err_{l2_err},
        linf_err_{linf_err} {}

  LinsysSolveResultImpl(vecxr const& solution, bool converged = true)
      : solution_{solution}, converged_{converged} {}
};

using LinsysSolveResult = StatusOr<LinsysSolveResultImpl>;

}  // namespace ax::math
