#pragma once
#include "ax/math/sparse_matrix/csr.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

class GeneralSparsePreconditioner_FSAI0 : public GeneralSparsePreconditionerBase {
public:
  GeneralSparsePreconditioner_FSAI0() = default;
  virtual ~GeneralSparsePreconditioner_FSAI0() override = default;

  void AnalyzePattern() override;

  void Factorize() override;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  GeneralPreconditionerKind GetKind() const override { return GeneralPreconditionerKind::FSAI0; }

  std::unique_ptr<RealCSRMatrix> fact_inv_;
};

}  // namespace ax::math