#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_BDCSVD : public DenseSolverBase {
public:
  result_type Solve(vecxr const& b, vecxr const& init_guess) final;
  void Analyse(problem_t const& problem) final;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kBDCSVD; }
private:
  Eigen::BDCSVD<matxxr> impl_;
};

}
