#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class PreconditionerIncompleteLU : public PreconditionerBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

  PreconditionerKind Kind() const final { return PreconditionerKind::kIncompleteLU; }

private:
  Eigen::IncompleteLUT<real, idx> impl_;
};
}  // namespace ax::math