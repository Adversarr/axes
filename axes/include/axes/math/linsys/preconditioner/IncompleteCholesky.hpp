#pragma once

#include "axes/math/linsys/preconditioner.hpp"
namespace ax::math {

class PreconditionerIncompleteCholesky : public PreconditionerBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

  PreconditionerKind Kind() const final { return PreconditionerKind::kIncompleteCholesky; }

private:
  Eigen::IncompleteCholesky<real, Eigen::Upper | Eigen::Lower, Eigen::AMDOrdering<idx>> impl_;
};

}  // namespace ax::math