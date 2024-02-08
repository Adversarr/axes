#pragma once

#include "axes/math/linsys/preconditioner.hpp"
namespace ax::math {

class PreconditionerDiagonal : public PreconditionerBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::DiagonalPreconditioner<real> impl_;
};
}  // namespace ax::math