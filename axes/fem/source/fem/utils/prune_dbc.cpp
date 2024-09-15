#include "ax/fem/utils/prune_dbc.hpp"

#include "ax/core/buffer/for_each.hpp"
#ifdef AX_HAS_CUDA
#  include "prune_dbc_gpu.hpp"
#endif
namespace ax::fem {

void do_prune_host(RealBufferView grad, ConstBufferView<VariableCondition> bc) {
  for_each(std::tuple{grad, bc}, [](Real& value, VariableCondition const& bc) {
    if (bc == VariableCondition::Dirichlet) {
      value = 0;
    }
  });
}

void PruneDirichletBc::Prune(RealBufferView grad) {
  auto bc = state_->GetCondition();
  if (!bc) {
    return;
  }

  auto device = bc->Device();
  if (grad.Device() != device) {
    throw std::runtime_error("input is not on the same device as the boundary condition");
  }

  if (device == BufferDevice::Host) {
    do_prune_host(grad, bc->ConstView());
  } else {
#ifdef AX_HAS_CUDA
    do_prune_gpu(grad, bc->ConstView());
#else
    throw std::runtime_error("CUDA is not enabled");
#endif
  }
}

}  // namespace ax::fem