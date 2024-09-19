#include "ax/fem/utils/prune_dbc.hpp"

#include "ax/core/buffer/for_each.hpp"
#include "ax/utils/cuda_helper.hpp"
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
    AX_THROW_RUNTIME_ERROR("input is not on the same device as the boundary condition");
  }

  if (device == BufferDevice::Host) {
    do_prune_host(grad, bc->ConstView());
  } else {
    AX_CUDA_CALL(do_prune_gpu(grad, bc->ConstView()));
  }
}

}  // namespace ax::fem