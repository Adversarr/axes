#pragma once
#include "ax/fem/state.hpp"

namespace ax::fem {

void do_prune_gpu(RealBufferView grad, ConstBufferView<VariableCondition> bc);

}  // namespace ax::fem
