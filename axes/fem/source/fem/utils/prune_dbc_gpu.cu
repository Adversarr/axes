#include "prune_dbc_gpu.hpp"

namespace ax::fem {

__global__ static void do_prune(Real *grad, const VariableCondition *bc,
                                const Real *bc_var, size_t total) {

  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= total)
    return;

  if (bc[idx] == VariableCondition::Dirichlet) {
    grad[idx] = bc_var[idx];
  }
}

void do_prune_gpu(RealBufferView grad, ConstBufferView<VariableCondition> bc,
                  ConstRealBufferView bc_var) {
  unsigned int block_size = 256;
  unsigned int num_blocks =
      static_cast<unsigned int>(prod(grad.Shape()) + block_size - 1) /
      block_size;
  do_prune<<<num_blocks, block_size>>>(grad.Data(), bc.Data(), bc_var.Data(),
                                       prod(grad.Shape()));
}

__global__ static void do_prune_hessian(RealBufferView block_val,
                                        ConstIntBufferView row_ptrs,
                                        ConstIntBufferView col_indices,
                                        ConstBufferView<VariableCondition> bc,
                                        size_t rows, size_t bs) {
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= rows)
    return;

  auto begin = row_ptrs(idx), end = row_ptrs(idx + 1);
  for (size_t bid = begin; bid < end; ++bid) {
    auto col = col_indices(bid);
    for (size_t l = 0; l < bs; ++l) {
      for (size_t k = 0; k < bs; ++k) {
        if (bc(k, idx) == VariableCondition::Dirichlet ||
            bc(l, col) == VariableCondition::Dirichlet) {
          block_val(k, l, bid) = 0;
        }
      }
    }
  }
}

void do_prune_hessian_gpu(math::RealBlockMatrix &hessian,
                          ConstBufferView<VariableCondition> bc) {
  unsigned int block_size = 256;
  size_t rows = hessian.RowPtrs()->Shape().X() - 1;
  unsigned int num_blocks =
      static_cast<unsigned int>(rows + block_size - 1) / block_size;

  do_prune_hessian<<<num_blocks, block_size>>>(
      hessian.Values()->View(), hessian.RowPtrs()->ConstView(),
      hessian.ColIndices()->ConstView(), bc, rows, hessian.BlockSize());
}

__global__ static void prepare_prune(Real *variables,
                                     const VariableCondition *bc,
                                     const Real *bc_var, size_t total) {

  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= total)
    return;

  if (bc[idx] == VariableCondition::Dirichlet) {
    variables[idx] = bc_var[idx];
  } else {
    variables[idx] = 0;
  }
}

void prepare_prune_gpu(ConstBufferView<VariableCondition> bc,
                       RealBufferView variables, ConstRealBufferView bc_var) {
  unsigned int block_size = 256;

  size_t total = prod(variables.Shape());
  unsigned int num_blocks =
      static_cast<unsigned int>(total + block_size - 1) / block_size;

  prepare_prune<<<num_blocks, block_size>>>(variables.Data(), bc.Data(),
                                            bc_var.Data(), total);
}

} // namespace ax::fem
