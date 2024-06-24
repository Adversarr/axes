#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_BDCSVD : public DenseSolverBase {
public:
  math::vecxr Solve(vecxr const& b) final;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kBDCSVD; }

private:
  Eigen::BDCSVD<matxxr> impl_;
};

}  // namespace ax::math
