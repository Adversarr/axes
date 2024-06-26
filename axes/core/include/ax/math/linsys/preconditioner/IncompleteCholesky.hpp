#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class PreconditionerIncompleteCholesky : public PreconditionerBase {
public:
  void Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

  PreconditionerKind Kind() const final { return PreconditionerKind::kIncompleteCholesky; }

private:
  // Eigen::IncompleteCholesky<real, Eigen::Upper | Eigen::Lower, Eigen::AMDOrdering<idx>> impl_;
  Eigen::IncompleteCholesky<real, Eigen::Upper | Eigen::Lower, Eigen::NaturalOrdering<idx>> impl_;
};

}  // namespace ax::math
