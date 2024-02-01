#pragma once

#include "axes/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

template <typename LinsysProblem> class LinsysSolverBase {
public:
  using result_type = LinsysSolveResult;
  using problem_t = LinsysProblem;

  virtual ~LinsysSolverBase() = default;

  /****************************** Prefactorize and checkings. ******************************/
  virtual Status Analyse(problem_t const& problem, utils::Opt const& options);

  /****************************** Solve ******************************/

  result_type Solve(problem_t& problem, utils::Opt const& options);
  virtual result_type Solve(problem_t& problem, vecxr const& init_guess, utils::Opt const& options);
  virtual result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options) = 0;
};

using DenseLinsysSolverBase = LinsysSolverBase<LinsysProblem_Dense>;
using SparseLinsysSolverBase = LinsysSolverBase<LinsysProblem_Sparse>;
using ImplicitLinsysSolverBase = LinsysSolverBase<LinsysProblem_Implicit>;


}  // namespace ax::math
