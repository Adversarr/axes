#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_ColPivHouseholderQR : public DenseSolverBase {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess);
  void Analyse(problem_t const& problem);
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kColPivHouseholderQR; }
private:
  Eigen::ColPivHouseholderQR<matxxr> impl_;
};

}  // namespace ax::math
