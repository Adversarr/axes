#include "ax/math/block_matrix/linsys/preconditioner/jacobi.hpp"

#include "./jacobi_impl.hpp"
#include "ax/core/buffer/create_default.hpp"

namespace ax::math {

void BlockPreconditioner_Jacobi::AnalyzePattern() {
  // Do nothing.
}

void BlockPreconditioner_Jacobi::Factorize() {
  AX_THROW_IF_NULLPTR(problem_, "BlockPreconditioner_Jacobi::Factorize: problem is nullptr.");

  auto device = problem_->A_.GetDevice();
  inv_diag_ = create_buffer<Real>(device, {problem_->A_.BlockSize(), problem_->A_.BlockedRows()});

  if (device == BufferDevice::Device) {
#ifdef AX_HAS_CUDA
    details::jacobi_precond_precompute_gpu(inv_diag_->View(), problem_->A_);
#else
    throw make_runtime_error("BlockPreconditioner_Jacobi::Factorize: CUDA is not enabled.");
#endif
  } else {
    details::jacobi_precond_precompute_cpu(inv_diag_->View(), problem_->A_);
  }
}

void BlockPreconditioner_Jacobi::Solve(ConstRealBufferView b, RealBufferView x) const {
  auto inv_diag = inv_diag_->View();
  AX_THROW_IF_FALSE(is_same_device(inv_diag, b, x),
                    "BlockPreconditioner_Jacobi::Solve: this, b and x must be on the same device.");

  auto rows = problem_->A_.BlockedRows();
  auto bs = problem_->A_.BlockSize();
  AX_THROW_IF_FALSE(
      b.Shape().X() == bs && b.Shape().Y() == rows && x.Shape().X() == bs && x.Shape().Y() == rows,
      "BlockPreconditioner_Jacobi::Solve: b and x must have the same shape as the block size and "
      "rows of the problem. got x {} b {}",
      x.Shape(), b.Shape());

  if (inv_diag.Device() == BufferDevice::Device) {
#ifdef AX_HAS_CUDA
    details::jacobi_precond_solve_gpu(x, b, inv_diag);
#else
    throw make_runtime_error("BlockPreconditioner_Jacobi::Solve: CUDA is not enabled.");
#endif
  } else {
    details::jacobi_precond_solve_cpu(x, b, inv_diag);
  }
}

}  // namespace ax::math