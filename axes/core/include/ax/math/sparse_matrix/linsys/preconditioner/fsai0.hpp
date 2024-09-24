#pragma once
#include "ax/math/sparse_matrix/csr.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

/**
 * @brief FSAI0 preconditioner
 * @note  Reference: Algorithms for Sparse Linear Systems, 11.2 Approximate Inverses Based on
 *        Frobenius Norm Minimization
 *
 *        The FSAI0 preconditioner is defined as the approximate inverse of the lower triangular
 *        part of the matrix. The approximate inverse is computed by solving a linear system.
 */
class GeneralSparsePreconditioner_FSAI0 : public GeneralSparsePreconditionerBase {
public:
  GeneralSparsePreconditioner_FSAI0() = default;
  ~GeneralSparsePreconditioner_FSAI0() override = default;

  void AnalyzePattern() override;

  void Factorize() override;

  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  GeneralPreconditionerKind GetKind() const override { return GeneralPreconditionerKind::FSAI0; }

  std::unique_ptr<RealCSRMatrix> fact_inv_;  ///< matrix g
  BufferPtr<Real> temp_;                     ///< stores the g.T x
};

}  // namespace ax::math