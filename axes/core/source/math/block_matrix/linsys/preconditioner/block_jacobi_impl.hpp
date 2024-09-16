#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"
#define AX_INVERSE_BLOCK_MAXSIZE 4
namespace ax::math::details {

void block_jacobi_precond_precompute_cpu(
  BufferView<Real> dst,
  const RealBlockMatrix& mat
);

void block_jacobi_precond_eval_cpu(
  BufferView<Real> dst,             // a view of [bs, rows, 0]
  ConstRealBufferView rhs,          // a view of [bs, rows, 0]
  ConstRealBufferView inv_diag      // a view of [bs, bs, rows]
);

void block_jacobi_precond_precompute_gpu(
  BufferView<Real> dst,
  const RealBlockMatrix& mat
);

void block_jacobi_precond_eval_gpu(
  BufferView<Real> dst,             // a view of [bs, rows, 0]
  ConstRealBufferView rhs,          // a view of [bs, rows, 0]
  ConstRealBufferView inv_diag      // a view of [bs, bs, rows]
);

namespace details {

template <size_t bs>
AX_HOST_DEVICE AX_FORCE_INLINE void do_inplace_inverse(Real* data) {
  // assume data is bs x bs.
  Eigen::Map<RealMatrix<bs, bs>> mat(data);
  mat = mat.inverse().eval();
}

template <size_t bs>
AX_HOST_DEVICE AX_FORCE_INLINE void do_matmul(Real* dst, const Real* A, const Real* b) {
  Eigen::Map<const RealMatrix<bs, bs>> mat_a(A);
  Eigen::Map<const RealMatrix<bs, 1>> vec_b(b);
  Eigen::Map<RealMatrix<bs, 1>> vec_dst(dst);
  vec_dst = mat_a * vec_b;
}

}  // namespace details

}  // namespace ax::math::details