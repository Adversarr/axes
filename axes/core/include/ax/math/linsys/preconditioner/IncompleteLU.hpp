#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class Preconditioner_IncompleteLU : public PreconditionerBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;
  RealMatrixX Solve(RealMatrixX const &b) override;

  PreconditionerKind GetKind() const final { return PreconditionerKind::IncompleteLU; }

private:
  Eigen::IncompleteLUT<Real, Index> impl_;
};
}  // namespace ax::math
