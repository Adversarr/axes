#pragma once
#include "ax/math/linsys/dense.hpp"
namespace ax::math {

class DenseSolver_ColPivHouseholderQR : public DenseSolverBase {
public:
  RealVectorX Solve(const RealVectorX &b) override;
  void Compute() override;
  virtual DenseSolverKind GetKind() const final { return DenseSolverKind::kColPivHouseholderQR; }
private:
  Eigen::ColPivHouseholderQR<RealMatrixX> impl_;
};

}  // namespace ax::math
