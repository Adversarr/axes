#include "ax/math/sparse_matrix/linsys/preconditioner/jacobi.hpp"

#include "./jacobi_impl.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/utils/cuda_helper.hpp"

namespace ax::math {

void GeneralSparsePreconditioner_Jacobi::AnalyzePattern() {
  // Do nothing.
}

void GeneralSparsePreconditioner_Jacobi::Factorize() {
  AX_THROW_IF_NULLPTR(mat_, "SparsePreconditioner_Jacobi::Factorize: problem is nullptr.");

  auto device = mat_->Device();
  auto& mat = *mat_;
  inv_diag_ = create_buffer<Real>(device, {mat.BlockSize(), mat.BlockedRows()});

  auto blocked = std::dynamic_pointer_cast<const RealBlockMatrix>(mat_);
  if (blocked) {
    if (device == BufferDevice::Device) {
      AX_CUDA_CALL(details::jacobi_precond_precompute_blocked_gpu(inv_diag_->View(), *blocked));
    } else {
      details::jacobi_precond_precompute_blocked_cpu(inv_diag_->View(), *blocked);
    }
  } else {
    // TODO: for other format, rather easy.
    AX_THROW_RUNTIME_ERROR("SparsePreconditioner_Jacobi::Factorize: unsupported matrix type.");
  }
}

void GeneralSparsePreconditioner_Jacobi::Solve(ConstRealBufferView b, RealBufferView x) const {
  auto inv_diag = inv_diag_->View();
  AX_THROW_IF_FALSE(
      is_same_device(inv_diag, b, x),
      "SparsePreconditioner_Jacobi::Solve: this, b and x must be on the same device.");

  auto rows = mat_->BlockedRows();
  auto bs = mat_->BlockSize();
  AX_THROW_IF_FALSE(
      b.Shape().X() == bs && b.Shape().Y() == rows && x.Shape().X() == bs && x.Shape().Y() == rows,
      "SparsePreconditioner_Jacobi::Solve: b and x must have the same shape as the block size and "
      "rows of the problem. got x {} b {}",
      x.Shape(), b.Shape());

  if (inv_diag.Device() == BufferDevice::Device) {
    AX_CUDA_CALL(details::jacobi_precond_solve_gpu(x, b, inv_diag));
  } else {
    details::jacobi_precond_solve_cpu(x, b, inv_diag);
  }
}

}  // namespace ax::math