#pragma once

#include "ax/math/sparse_matrix/linsys/preconditioner.hpp"

namespace ax::math {

/**
 * @brief Incomplete LU preconditioner, based on SuperLU or Eigen.
 * 
 */
class GeneralSparsePreconditioner_ILU final : public GeneralSparsePreconditionerBase {
public:
  GeneralSparsePreconditioner_ILU();
  virtual ~GeneralSparsePreconditioner_ILU();

  virtual void Solve(ConstRealBufferView b, RealBufferView x) const final;

  virtual void AnalyzePattern() final;

  virtual void Factorize() final;

  virtual GeneralPreconditionerKind GetKind() const final { return GeneralPreconditionerKind::ILU; }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}