#include "ax/math/block_matrix/linsys/preconditioner/block_jacobi.hpp"
#ifdef AX_HAS_CUDA
#  include "ax/core/buffer/device_buffer.cuh"
#endif
#include "ax/core/buffer/host_buffer.hpp"
#include "block_jacobi_impl.hpp"

namespace ax::math {

void BlockPreconditioner_BlockJacobi::AnalyzePattern() {
  // Do nothing.
}

void BlockPreconditioner_BlockJacobi::Factorize() {
  AX_THROW_IF_NULLPTR(problem_, "problem_ is nullptr.");
  BufferDevice device = problem_->A_.GetDevice();

#ifndef AX_HAS_CUDA
  if (device == BufferDevice::Device) {
    throw make_runtime_error("BlockPreconditioner_BlockJacobi::Factorize: CUDA is not enabled.");
  }
#endif

  size_t b_rows = problem_->A_.BlockedRows();
  size_t b_cols = problem_->A_.BlockedCols();
  size_t bs = problem_->A_.BlockSize();

  if (b_rows != b_cols) {
    throw make_invalid_argument("BlockPreconditioner_BlockJacobi::Factorize: A is not square.");
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
    details::block_jacobi_precond_precompute_cpu(inv_diag_->View(), problem_->A_);
  } else {
#ifdef AX_HAS_CUDA
    details::block_jacobi_precond_precompute_gpu(inv_diag_->View(), problem_->A_);
#else
    throw make_runtime_error("BlockPreconditioner_BlockJacobi::Factorize: CUDA is not enabled.");
#endif
  }
}

void BlockPreconditioner_BlockJacobi::Solve(ConstRealBufferView b, RealBufferView x) const {
  auto device = problem_->A_.GetDevice();
  AX_THROW_IF_NULLPTR(inv_diag_, "inv_diag_ is null. compute inv_diag_ first.");
  size_t bs = problem_->A_.BlockSize();
  size_t rows = problem_->A_.BlockedRows();
  AX_THROW_IF_FALSE(
      b.Shape().X() == bs && b.Shape().Y() == rows && x.Shape().X() == bs && x.Shape().Y() == rows,
      "b and x must have the same shape as the block size and rows of the problem.");

  if (device == BufferDevice::Host) {
    details::block_jacobi_precond_eval_cpu(x, b, inv_diag_->View());
  } else {
#ifdef AX_HAS_CUDA
    details::block_jacobi_precond_eval_gpu(x, b, inv_diag_->View());
#else
    throw make_runtime_error("BlockPreconditioner_BlockJacobi::Solve: CUDA is not enabled.");
#endif
  }
}

};  // namespace ax::math