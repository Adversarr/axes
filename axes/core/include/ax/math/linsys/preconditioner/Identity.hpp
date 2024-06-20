#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {
class Preconditioner_Identity : public PreconditionerBase {
public:
  void Analyse(LinsysProblem_Sparse const &problem) override;

  vecxr Solve(vecxr const &b, vecxr const &x0) override;

  PreconditionerKind GetKind() const final { return PreconditionerKind::kIdentity; }
};
}  // namespace ax::math