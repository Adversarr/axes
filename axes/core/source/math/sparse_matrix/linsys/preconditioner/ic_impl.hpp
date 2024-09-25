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

struct ImplIcBsrGpu final : public GeneralSparsePreconditioner_IncompleteCholesky::Impl {
  void AnalyzePattern() override;
  void Factorize() override;
  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  BufferDevice Device() const override { return BufferDevice::Device; }

  explicit ImplIcBsrGpu(ConstRealSparseMatrixPtr mat);
  ImplIcBsrGpu(const ImplIcBsrGpu&) = delete;
  ImplIcBsrGpu& operator=(const ImplIcBsrGpu&) = delete;
  ImplIcBsrGpu(ImplIcBsrGpu&&) = delete;
  ImplIcBsrGpu& operator=(ImplIcBsrGpu&&) = delete;
  ~ImplIcBsrGpu();

  ConstRealSparseMatrixPtr mat_;
  BufferPtr<Real> mid_result_;  // stores L^{-T} b
  void* buffer_ = nullptr;      // the temporary buffer for cusparse
  void* descr_M_ = nullptr;     // the matrix descriptor
  void* descr_L_ = nullptr;     // the buffer for M
  void* info_M_ = nullptr;      // the info for M
  void* info_L_ = nullptr;      // the info for L
  void* info_Lt_ = nullptr;     // the info for L^T
};

struct ImplIcCsrGpu final : public GeneralSparsePreconditioner_IncompleteCholesky::Impl {
  void AnalyzePattern() override;
  void Factorize() override;
  void Solve(ConstRealBufferView b, RealBufferView x) const override;

  BufferDevice Device() const override { return BufferDevice::Device; }

  explicit ImplIcCsrGpu(ConstRealSparseMatrixPtr mat);
  ImplIcCsrGpu(const ImplIcCsrGpu&) = delete;
  ImplIcCsrGpu& operator=(const ImplIcCsrGpu&) = delete;
  ImplIcCsrGpu(ImplIcCsrGpu&&) = delete;
  ImplIcCsrGpu& operator=(ImplIcCsrGpu&&) = delete;
  ~ImplIcCsrGpu();

  ConstRealSparseMatrixPtr mat_;
  std::unique_ptr<math::RealCSRMatrix> L_;

  BufferPtr<Real> mid_result_;  // stores L^{-T} b
  std::shared_ptr<void> b_descr_;
  std::shared_ptr<void> x_descr_;
  std::shared_ptr<void> temp_descr_;
  std::shared_ptr<void> spsv_descr_;
  std::shared_ptr<void> spsv_t_descr_;
  // std::shared_ptr<void>(

  void* external_buffer_l_{nullptr};
  void* external_buffer_lt_{nullptr};
};

}  // namespace ax::math