#pragma once

#include "axes/utils/opt.hpp"
#include "common.hpp"

namespace ax::math {

/**
 * @brief Base class for linear system solvers.
 * 
 * This class provides a base implementation for solving linear systems of equations.
 * It defines the common interface and functionality that all linear system solvers should have.
 * 
 * @tparam LinsysProblem The type of the linear system problem.
 */
template <typename LinsysProblem> class LinsysSolverBase : public utils::Tunable {
public:
  using result_type = LinsysSolveResult;
  using problem_t = LinsysProblem;

  /**
   * @brief Destructor.
   */
  virtual ~LinsysSolverBase() = default;

  /**
   * @brief Analyzes the linear system problem.
   * 
   * This method performs any necessary pre-processing or factorization steps on the linear system problem.
   * It should be called before calling the Solve() method.
   * 
   * @param problem The linear system problem to be analyzed.
   * @return The status of the analysis operation.
   */
  virtual Status Analyse(problem_t const& problem) = 0;

  /**
   * @brief Solves the linear system problem.
   * 
   * This method solves the linear system problem given the right-hand side vector and an optional initial guess.
   * It should be called after calling the Analyse() method.
   * 
   * @param b The right-hand side vector of the linear system.
   * @param init_guess An optional initial guess for the solution.
   * @return The result of the solve operation.
   */
  virtual result_type Solve(vecxr const& b, vecxr const& init_guess) = 0;

  /**
   * @brief Solves the linear system problem with default initialization.
   * 
   * This method solves the linear system problem using the default initialization.
   * It calls the Analyse() method internally before calling the Solve() method.
   * 
   * @param problem The linear system problem to be solved.
   * @param init_guess An optional initial guess for the solution.
   * @return The result of the solve operation.
   */
  virtual result_type SolveProblem(problem_t const& problem, vecxr const& init_guess = {}) {
    auto status = Analyse(problem);
    if (!status.ok()) {
      return status;
    }
    return Solve(problem.b_, init_guess);
  }
};

}  // namespace ax::math
