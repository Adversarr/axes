#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class Preconditioner_IncompleteCholesky : public PreconditionerBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;

  matxxr Solve(matxxr const &b) override;

  PreconditionerKind GetKind() const final { return PreconditionerKind::kIncompleteCholesky; }

private:
  Eigen::IncompleteCholesky<real, Eigen::Lower, Eigen::NaturalOrdering<Index>> impl_;
};

}  // namespace ax::math
