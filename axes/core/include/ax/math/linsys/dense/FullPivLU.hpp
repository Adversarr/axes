#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_FullPivLU : public DenseSolverBase {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess);
  void Analyse(problem_t const& problem);

private:
  Eigen::FullPivLU<matxxr> impl_;
};
}  // namespace ax::math
