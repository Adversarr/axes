#pragma once

#include "axes/math/linsys/preconditioner.hpp"
namespace ax::math {
class PreconditionerIdentity : public PreconditionerBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;
};
}  // namespace ax::math