#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class Preconditioner_IncompleteLU : public PreconditionerBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;
  vecxr Solve(vecxr const &b) override;

  PreconditionerKind GetKind() const final { return PreconditionerKind::kIncompleteLU; }

private:
  Eigen::IncompleteLUT<real, idx> impl_;
};
}  // namespace ax::math
