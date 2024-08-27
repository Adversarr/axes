#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class Preconditioner_IncompleteCholesky : public PreconditionerBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  RealMatrixX Solve(RealMatrixX const &b) override;

  PreconditionerKind GetKind() const final { return PreconditionerKind::IncompleteCholesky; }

private:
  Eigen::IncompleteCholesky<Real, Eigen::Lower, Eigen::NaturalOrdering<Index>> impl_;
};

}  // namespace ax::math
