#include "ax/math/sparse_matrix/linsys/preconditioner/block_jacobi.hpp"
#ifdef AX_HAS_CUDA
#  include "ax/core/buffer/device_buffer.cuh"
#endif
#include "ax/core/buffer/host_buffer.hpp"
#include "block_jacobi_impl.hpp"

namespace ax::math {

void GeneralSparsePreconditioner_BlockJacobi::AnalyzePattern() {
  // Do nothing.
}

void GeneralSparsePreconditioner_BlockJacobi::Factorize() {
  AX_THROW_IF_NULLPTR(mat_, "problem_ is nullptr.");
  BufferDevice device = mat_->Device();

#ifndef AX_HAS_CUDA
  if (device == BufferDevice::Device) {
    AX_THROW_RUNTIME_ERROR("SparsePreconditioner_BlockJacobi::Factorize: CUDA is not enabled.");
  }
#endif

  size_t b_rows = mat_->BlockedRows();
  size_t b_cols = mat_->BlockedCols();
  size_t bs = mat_->BlockSize();

  if (bs == 1) {
    AX_WARN("SparsePreconditioner_BlockJacobi::Factorize: Block size is 1, Prefer Jacobi!");
  }

  if (b_rows != b_cols) {
    throw make_invalid_argument("SparsePreconditioner_BlockJacobi::Factorize: A is not square.");
  }

  if (!inv_diag_) {
    if (device == BufferDevice::Host) {
      inv_diag_ = HostBuffer<Real>::Create({bs, bs, b_rows});
    } else {
#ifdef AX_HAS_CUDA
      inv_diag_ = DeviceBuffer<Real>::Create({bs, bs, b_rows});
#endif
    }
  }

  AX_CHECK(inv_diag_->IsContinuous(),
           "(InternalError) inv_diag_ must be continuous for memory mapping.");

  // Precompute the inverse of the diagonal blocks of A, and store them in inv_diag_.
  if (device == BufferDevice::Host) {
    details::block_jacobi_precond_precompute_cpu(inv_diag_->View(),
                                                 *static_cast<const RealBlockMatrix*>(mat_.get()));
  } else {
#ifdef AX_HAS_CUDA
    details::block_jacobi_precond_precompute_gpu(inv_diag_->View(),
                                                 *static_cast<const RealBlockMatrix*>(mat_.get()));
#else
    AX_THROW_RUNTIME_ERROR("SparsePreconditioner_BlockJacobi::Factorize: CUDA is not enabled.");
#endif
  }
}

void GeneralSparsePreconditioner_BlockJacobi::Solve(ConstRealBufferView b, RealBufferView x) const {
  auto device = mat_->Device();
  AX_THROW_IF_NULLPTR(inv_diag_, "inv_diag_ is null. compute inv_diag_ first.");
  size_t bs = mat_->BlockSize();
  size_t rows = mat_->BlockedRows();
  AX_THROW_IF_FALSE(
      b.Shape().X() == bs && b.Shape().Y() == rows && x.Shape().X() == bs && x.Shape().Y() == rows,
      "b and x must have the same shape as the block size and rows of the problem.");

  if (device == BufferDevice::Host) {
    details::block_jacobi_precond_eval_cpu(x, b, inv_diag_->View());
  } else {
#ifdef AX_HAS_CUDA
    details::block_jacobi_precond_eval_gpu(x, b, inv_diag_->View());
#else
    AX_THROW_RUNTIME_ERROR("SparsePreconditioner_BlockJacobi::Solve: CUDA is not enabled.");
#endif
  }
}

};  // namespace ax::math