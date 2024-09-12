#pragma once
#include "ax/core/buffer/buffer_view.hpp"
#include "ax/math/block_matrix/block_matrix.hpp"
#define AX_INVERSE_BLOCK_MAXSIZE 4
namespace ax::math::details {

void block_jacobi_precond_precompute_cpu(
  BufferView<Real> dst,
  const RealBlockMatrix& mat,
  void* mat_desc_type_erased
);

void block_jacobi_precond_eval_cpu(
  BufferView<Real> dst,             // a view of [bs, rows, 0]
  ConstRealBufferView rhs,          // a view of [bs, rows, 0]
  ConstRealBufferView inv_diag,     // a view of [bs, bs, rows]
  void* mat_desc_type_erased
);

void block_jacobi_precond_precompute_gpu(
  BufferView<Real> dst,
  const RealBlockMatrix& mat,
  void* mat_desc_type_erased
);

namespace details {

template <size_t bs>
void do_inplace_inverse(Real* data) {
  // assume data is bs x bs.
  Eigen::Map<RealMatrix<bs, bs>> mat(data);
  RealMatrix<bs, bs> mat_copy = mat.eval();
  mat = mat.inverse().eval();
}

}  // namespace details

}  // namespace ax::math::details