#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class PreconditionerDiagonal : public PreconditionerBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

  PreconditionerKind Kind() const final { return PreconditionerKind::kDiagonal; }

private:
  Eigen::DiagonalPreconditioner<real> impl_;
};
}  // namespace ax::math