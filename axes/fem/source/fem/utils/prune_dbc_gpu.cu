#include "prune_dbc_gpu.hpp"

namespace ax::fem {

__global__ static void do_prune(Real *grad, const VariableCondition *bc,
                                size_t total) {

  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= total)
    return;

  if (bc[idx] == VariableCondition::Dirichlet) {
    grad[idx] = 0.0;
  }
}

void do_prune_gpu(RealBufferView grad, ConstBufferView<VariableCondition> bc) {
  unsigned int block_size = 256;
  unsigned int num_blocks = (prod(grad.Shape()) + block_size - 1) / block_size;
  do_prune<<<num_blocks, block_size>>>(grad.Data(), bc.Data(),
                                       prod(grad.Shape()));
}

} // namespace ax::fem
