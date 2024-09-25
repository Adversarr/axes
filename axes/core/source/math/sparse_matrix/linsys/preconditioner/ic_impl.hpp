#pragma once
#include "ax/math/linsys/preconditioner/IncompleteCholesky.hpp"
#include "ax/math/sparse_matrix/csr.hpp"
#include "ax/math/sparse_matrix/linsys/preconditioner/ic.hpp"

namespace ax::math {

struct GeneralSparsePreconditioner_IncompleteCholesky::Impl {
  virtual ~Impl() = default;
  virtual void AnalyzePattern() = 0;
  virtual void Factorize() = 0;
  virtual void Solve(ConstRealBufferView b, RealBufferView x) const = 0;
  virtual BufferDevice Device() const = 0;
};

struct ImplIcCpu final : public GeneralSparsePreconditioner_IncompleteCholesky::Impl {
  void AnalyzePattern() override;
  void Factorize() override;
  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  BufferDevice Device() const override { return BufferDevice::Host; }

  explicit ImplIcCpu(ConstRealSparseMatrixPtr mat) : mat_(mat) {}

  ConstRealSparseMatrixPtr mat_;
  math::RealSparseMatrix mat_eigen_;
  Eigen::IncompleteCholesky<Real, Eigen::Upper | Eigen::Lower, Eigen::AMDOrdering<SparseIndex>> ic_;
};

struct ImplIcGpu final : public GeneralSparsePreconditioner_IncompleteCholesky::Impl {
  void AnalyzePattern() override;
  void Factorize() override;
  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  BufferDevice Device() const override { return BufferDevice::Device; }

  BufferPtr<Real> mid_result_;  // stores L^{-T} b
  math::RealCSRMatrix L_;       // stores the incomplete Cholesky factorization
};

}  // namespace ax::math