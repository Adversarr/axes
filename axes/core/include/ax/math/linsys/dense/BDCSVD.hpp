#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_BDCSVD : public DenseSolverBase {
public:
  RealVectorX Solve(RealVectorX const& b) final;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::BDCSVD; }

private:
  Eigen::BDCSVD<RealMatrixX> impl_;
};

}  // namespace ax::math
