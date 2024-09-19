#pragma once
#include "ax/fem/state.hpp"
#include "ax/math/sparse_matrix/block_matrix.hpp"

namespace ax::fem {

void do_prune_gpu(RealBufferView grad, ConstBufferView<VariableCondition> bc, ConstRealBufferView bc_var);

void do_prune_hessian_gpu(math::RealBlockMatrix& hessian, ConstBufferView<VariableCondition> bc);

void prepare_prune_gpu(ConstBufferView<VariableCondition> bc, RealBufferView variables, ConstRealBufferView bc_var);

}  // namespace ax::fem
