#include "gather_gpu.cuh"

namespace ax::math {

static __global__ void do_gather_1d(
  ConstRealBufferView src, RealBufferView dst,
  ConstRealBufferView weights, ConstSizeBufferView row_entries,
  ConstSizeBufferView col_indices, Real alpha, Real beta, size_t n_output) {
  // do gather in the first dimension
  const size_t row = blockIdx.x * blockDim.x + threadIdx.x;
  if (row >= n_output) {
    return;
  }

  const size_t start = row_entries(row);
  const size_t end = row_entries(row + 1);

  Real sum = 0;
  for (size_t i = start; i < end; ++i) {
    const size_t col = col_indices(i);
    sum += src(col) * weights(i);
  }

  dst(row) = alpha * sum + beta * dst(row);
}

static __global__ void do_gather_2d(
  ConstRealBufferView src, RealBufferView dst,
  ConstRealBufferView weights, ConstSizeBufferView row_entries,
  ConstSizeBufferView col_indices, Real alpha, Real beta,
  size_t n_output) {
  // do gather in the second dimension
  const size_t to = blockIdx.y * blockDim.y + threadIdx.y;
  const size_t sub = threadIdx.x;
  if (to >= n_output) {
    return;
  }

  const size_t start = row_entries(to);
  const size_t end = row_entries(to + 1);
  Real sum = 0;
  for (size_t i = start; i < end; ++i) {
    const size_t from = col_indices(i);
    sum += src(sub, from) * weights(i);
  }
  dst(sub, to) = alpha * sum + beta * dst(sub, to);
}

static __global__ void do_gather_3d(
  ConstRealBufferView src, RealBufferView dst,
  ConstRealBufferView weights, ConstSizeBufferView row_entries,
  ConstSizeBufferView col_indices, Real alpha, Real beta,
  size_t n_output) {
  // do gather in the third dimension
  const size_t to = blockIdx.z;
  const size_t sub_x = threadIdx.x;
  const size_t sub_y = threadIdx.y;
  if (to >= n_output) {
    return;
  }

  const size_t start = row_entries(to);
  const size_t end = row_entries(to + 1);
  Real sum = 0;
  for (size_t i = start; i < end; ++i) {
    const size_t from = col_indices(i);
    sum += src(sub_x, sub_y, from) * weights(i);
  }
  dst(sub_x, sub_y, to) = alpha * sum + beta * dst(sub_x, sub_y, to);
}

void gather_device(ConstRealBufferView src, RealBufferView dst,
                   ConstRealBufferView weights, ConstSizeBufferView row_entries,
                   ConstSizeBufferView col_indices, Real alpha, Real beta,
                   size_t n_output, size_t gather_dim) {
  // ...
  // no check

  if (gather_dim == 0) {
    unsigned int block_size = 256;
    unsigned int n_block = (n_output + block_size - 1) / block_size;
    do_gather_1d<<<n_block, block_size>>>(src, dst, weights, row_entries, col_indices, alpha, beta, n_output);
  } else if (gather_dim == 1) {
    // 2D gather
    size_t dim_x = src.Shape().X();
    dim3 block_size(static_cast<unsigned int>(dim_x), 32);
    dim3 grid_size(1, (n_output + block_size.y - 1) / block_size.y);
    do_gather_2d<<<grid_size, block_size>>>(src, dst, weights, row_entries,
                                            col_indices, alpha, beta, n_output);
  } else {
    // 3D gather.
    size_t dim_x = src.Shape().X(), dim_y = src.Shape().Y();
    dim3 block_size(static_cast<unsigned int>(dim_x), static_cast<unsigned int>(dim_y));
    dim3 grid_size(1, 1, n_output);
    do_gather_3d<<<grid_size, block_size>>>(src, dst, weights, row_entries,
                                            col_indices, alpha, beta, n_output);
  }
}

} // namespace ax::math