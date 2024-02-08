#pragma once

#include "axes/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

template <typename LinsysProblem> class LinsysSolverBase : public utils::Tunable {
public:
  using result_type = LinsysSolveResult;
  using problem_t = LinsysProblem;

  virtual ~LinsysSolverBase() = default;

  /****************************** Prefactorize and checkings. ******************************/
  virtual Status Analyse(problem_t const& problem) = 0;

  /****************************** Solve ******************************/
  virtual result_type SolveProblem(problem_t const& problem, vecxr const& init_guess = {}) {
    auto status = Analyse(problem);
    if (!status.ok()) {
      return status;
    }
    return Solve(problem.b_, init_guess);
  }

  virtual result_type Solve(vecxr const& b, vecxr const& init_guess) = 0;
};

}  // namespace ax::math
