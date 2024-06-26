
#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_LDLT final : public DenseSolverBase {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess) final;
  void Analyse(problem_t const& problem) final;

private:
  Eigen::LDLT<matxxr> impl_;
};
}  // namespace ax::math
