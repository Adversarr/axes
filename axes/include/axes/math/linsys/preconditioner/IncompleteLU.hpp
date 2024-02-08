#pragma once

#include "axes/math/linsys/preconditioner.hpp"
namespace ax::math {

class PreconditionerIncompleteLU : public PreconditionerBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

private:
  Eigen::IncompleteLUT<real, idx> impl_;
};
}  // namespace ax::math