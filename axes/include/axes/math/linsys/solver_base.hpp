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
  virtual Status Analyse(problem_t const& problem, utils::Opt const& options) = 0;

  /****************************** Solve ******************************/
  result_type Solve(problem_t const& problem, utils::Opt const& options) {
    return Solve(problem, vecxr{}, options);
  }

  virtual result_type Solve(problem_t const& problem, vecxr const& init_guess, utils::Opt const& options) {
    auto status = Analyse(problem, options);
    if (!status.ok()) {
      return status;
    }
    return Solve(problem.b_, init_guess, options);
  }

  virtual result_type Solve(vecxr const& b, vecxr const& init_guess, utils::Opt const& options) = 0;
};



}  // namespace ax::math
