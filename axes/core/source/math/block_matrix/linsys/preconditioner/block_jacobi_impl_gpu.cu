#include "ax/core/excepts.hpp"
#include "block_jacobi_impl.hpp"

namespace ax::math::details {

__global__ void block_jacobi_precond_precompute_kernel(
    RealBufferView inv_diag, ConstRealBufferView value,
    BufferView<const int> row_ptr, BufferView<const int> col_idx, size_t rows,
    size_t bs) {
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= rows) {
    return;
  }

  for (size_t li = 0; li < bs; ++li) {
    for (size_t lj = 0; lj < bs; ++lj) {
      inv_diag(li, lj, i) = (li == lj) ? 1.0 : 0.0;
    }
  }

  const size_t row_start = row_ptr(i);
  const size_t row_end = row_ptr(i + 1);
  for (size_t bid = row_start; bid < row_end; ++bid) {
    const size_t j = col_idx(bid);
    if (j == i) {
      // copy the diagonal block
      for (size_t li = 0; li < bs; ++li) {
        for (size_t lj = 0; lj < bs; ++lj) {
          inv_diag(li, lj, i) = value(li, lj, bid);
        }
      }
    }
  }
  Real *inv_diag_ptr = inv_diag.Offset(0, 0, i);
  if (bs == 2) {
    details::do_inplace_inverse<2>(inv_diag_ptr);
  } else if (bs == 3) {
    details::do_inplace_inverse<3>(inv_diag_ptr);
  } else if (bs == 4) {
    details::do_inplace_inverse<4>(inv_diag_ptr);
  } else {
    assert(false && "Invalid Input!");
  }
}

void block_jacobi_precond_precompute_gpu(BufferView<Real> dst,
                                         const RealBlockMatrix &mat) {
  const size_t rows = mat.BlockedRows();
  const size_t bs = mat.BlockSize();
  if (bs > AX_INVERSE_BLOCK_MAXSIZE) {
    throw make_runtime_error("Block size is too large! got {} > {}", bs,
                             AX_INVERSE_BLOCK_MAXSIZE);
  }

  const size_t block_size = 256;
  const size_t grid_size = (rows + block_size - 1) / block_size;
  block_jacobi_precond_precompute_kernel<<<grid_size, block_size>>>(
      dst, mat.BlockValuesView(), mat.BlockRowPtrsView(),
      mat.BlockColIndicesView(), rows, bs);
}

__global__ void block_jacobi_precond_eval_kernel(RealBufferView dst,
                                                 ConstRealBufferView rhs,
                                                 ConstRealBufferView inv_diag,
                                                 size_t rows, size_t bs) {
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= rows) {
    return;
  }

  Real *dst_ptr = dst.Offset(0, i);
  const Real *inv_diag_ptr = inv_diag.Offset(0, 0, i);
  const Real *rhs_ptr = rhs.Offset(0, i);

  if (bs == 2) {
    details::do_matmul<2>(dst_ptr, inv_diag_ptr, rhs_ptr);
  } else if (bs == 3) {
    details::do_matmul<3>(dst_ptr, inv_diag_ptr, rhs_ptr);
  } else if (bs == 4) {
    details::do_matmul<4>(dst_ptr, inv_diag_ptr, rhs_ptr);
  }
}

void block_jacobi_precond_eval_gpu(
    BufferView<Real> dst,         // a view of [bs, rows, 0]
    ConstRealBufferView rhs,      // a view of [bs, rows, 0]
    ConstRealBufferView inv_diag  // a view of [bs, bs, rows]
) {
  const size_t rows = rhs.Shape().Y();
  const size_t bs = rhs.Shape().X();
  const size_t block_size = 256;
  const size_t grid_size = (rows + block_size - 1) / block_size;
  block_jacobi_precond_eval_kernel<<<grid_size, block_size>>>(
      dst, rhs, inv_diag, rows, bs);
}

} // namespace ax::math::details