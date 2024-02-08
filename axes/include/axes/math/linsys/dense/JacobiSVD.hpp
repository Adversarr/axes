
#pragma once
#include "axes/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_JacobiSVD : public DenseSolverBase {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess);
  Status Analyse(problem_t const& problem);

private:
  Eigen::JacobiSVD<matxxr> impl_;
};

}  // namespace ax::math
