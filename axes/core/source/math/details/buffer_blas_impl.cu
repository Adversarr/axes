#include "buffer_blas_impl.hpp"

namespace ax::math::buffer_blas {

static __global__ void do_emul_gpu_impl(ConstRealBufferView x, RealBufferView y) {
  size_t linear = blockIdx.x * blockDim.x + threadIdx.x;
  size_t total = prod(x.Shape());
  if (linear > total) {
    return;
  }

  size_t i = linear % x.Shape().X();
  size_t j = x.Shape().Y() > 0 ? (linear / x.Shape().X()) % x.Shape().Y() : 0;
  size_t k = x.Shape().Z() > 0 ? linear / (x.Shape().X() * x.Shape().Y()) : 0;

  y(i, j, k) *= x(i, j, k);
}

static __global__ void do_ediv_gpu_impl(ConstRealBufferView x, RealBufferView y) {
  size_t linear = blockIdx.x * blockDim.x + threadIdx.x;
  size_t total = prod(x.Shape());
  if (linear > total) {
    return;
  }

  size_t i = linear % x.Shape().X();
  size_t j = x.Shape().Y() > 0 ? (linear / x.Shape().X()) % x.Shape().Y() : 0;
  size_t k = x.Shape().Z() > 0 ? linear / (x.Shape().X() * x.Shape().Y()) : 0;

  y(i, j, k) /= x(i, j, k);
}

void do_emul_gpu(ConstRealBufferView x, RealBufferView y) {
  size_t total = prod(x.Shape());
  ui32 block_size = 256;
  ui32 block_count = (total + block_size - 1) / block_size;

  do_emul_gpu_impl<<<block_count, block_size>>>(x, y);
}

void do_ediv_gpu(ConstRealBufferView x, RealBufferView y) {
  size_t total = prod(x.Shape());
  ui32 block_size = 256;
  ui32 block_count = (total + block_size - 1) / block_size;

  do_ediv_gpu_impl<<<block_count, block_size>>>(x, y);
}

}  // namespace ax::math::buffer_blas