#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_BDCSVD : public DenseSolverBase {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess);
  Status Analyse(problem_t const& problem);

private:
  Eigen::BDCSVD<matxxr> impl_;
};

}
