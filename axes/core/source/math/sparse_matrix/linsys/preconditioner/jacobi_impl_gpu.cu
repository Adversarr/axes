#include "ax/core/excepts.hpp"
#include "jacobi_impl.hpp"

namespace ax::math::details {

__global__ void jacobi_precond_precompute_kernel(RealBufferView inv_diag,
                                                 ConstRealBufferView value,
                                                 BufferView<const int> row_ptr,
                                                 BufferView<const int> col_idx,
                                                 size_t rows) {
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= rows) {
    return;
  }

  const size_t bs = value.Shape().X();
  const int row_start = row_ptr(i);
  const int row_end = row_ptr(i + 1);

  for (size_t li = 0; li < bs; ++li) {
    inv_diag(li, i) = 1.0;
  }

  for (int bid = row_start; bid < row_end; ++bid) {
    const size_t j = col_idx(static_cast<size_t>(bid));
    if (j == i) {
      // copy the diagonal block
      for (size_t li = 0; li < bs; ++li) {
        inv_diag(li, i) = 1.0 / value(li, li, bid);
      }
    }
  }
}

void jacobi_precond_precompute_blocked_gpu(RealBufferView dst,
                                   const RealBlockMatrix &mat) {

  const size_t rows = mat.BlockedRows();
  const unsigned int block_size = 256;
  const unsigned int grid_size =
      static_cast<ui32>(rows + block_size - 1) / block_size;

  auto bv = mat.Values()->ConstView();
  auto br = mat.RowPtrs()->View();
  auto bc = mat.ColIndices()->View();

  jacobi_precond_precompute_kernel<<<grid_size, block_size>>>(
      dst, bv, br,
      bc, rows);
}

__global__ void jacobi_precond_solve_kernel(RealBufferView dst,
                                            ConstRealBufferView rhs,
                                            ConstRealBufferView inv_diag,
                                            size_t rows) {
  const size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  const size_t j = blockIdx.y * blockDim.y + threadIdx.y;
  const size_t bs = inv_diag.Shape().X();
  if (i >= rows || j >= bs) {
    return;
  }

  dst(j, i) = rhs(j, i) * inv_diag(j, i);
}

void jacobi_precond_solve_gpu(RealBufferView dst, ConstRealBufferView rhs,
                              ConstRealBufferView inv_diag) {
  const size_t rows = rhs.Shape().Y();
  const size_t bs = inv_diag.Shape().X();
  const dim3 block_size(32, bs);
  const dim3 grid_size((rows + block_size.x - 1) / block_size.x, 1);
  jacobi_precond_solve_kernel<<<grid_size, block_size>>>(dst, rhs, inv_diag,
                                                         rows);
}
} // namespace ax::math::details