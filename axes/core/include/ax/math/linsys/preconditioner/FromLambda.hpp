#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {
class Preconditioner_FromLambda : public PreconditionerBase {
public:
  void AnalyzePattern() override{}
  void Factorize() override{}
  RealMatrixX Solve(RealMatrixX const &b) override { 
    AX_THROW_IF_NULL(lambda_, "Lambda is not set.");
    return lambda_(b);
  }

  PreconditionerKind GetKind() const final { return PreconditionerKind::FromLambda; }

  std::function<RealMatrixX(RealMatrixX const&)> lambda_;
};
}  // namespace ax::math
