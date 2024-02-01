#pragma once

#include "axes/math/common.hpp"
#include "axes/math/sparse.hpp"

namespace ax::math {

/****************************** Problem Definition ******************************/

// NOTE: There are 3 kinds of Linear Systems:
//  1. Dense Matrix.
//  2. Sparse Matrix
//  3. Implicit

struct LinsysProblem_Dense {
  // Problem Description
  matxxr const& A_;
  vecxr const& b_;
  bool is_spsd_;
};

struct LinsysProblem_Sparse {
  // Problem Description
  sp_matxxr const& A_;
  vecxr const& b_;
  bool is_spsd_;

  // For Iterative Solvers: Solution Requirement
  real l2_tol_;
  real linf_tol_;

  // Additional checkers.
  std::function<bool(vecxr const&)> converge_residual_;
  std::function<bool(vecxr const&)> converge_solution_;
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

  // For iterative solver
  idx num_iter_;
  vecxr residual_;

  // May be not set?
  real l2_err_;
  real linf_err_;
};

using LinsysSolveResult = StatusOr<LinsysSolveResultImpl>;

}  // namespace ax::math
