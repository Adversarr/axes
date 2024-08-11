#pragma once

#include "ax/math/linsys/preconditioner.hpp"
namespace ax::math {

class Preconditioner_Diagonal : public PreconditionerBase {
public:
  void AnalyzePattern() override;
  void Factorize() override;
  matxxr Solve(matxxr const &b) override;

  PreconditionerKind GetKind() const final { return PreconditionerKind::kDiagonal; }

private:
  Eigen::DiagonalPreconditioner<real> impl_;
};
}  // namespace ax::math
