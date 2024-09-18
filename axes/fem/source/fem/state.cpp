#include "ax/fem/state.hpp"

#include "ax/core/buffer/copy.hpp"
#include "ax/core/buffer/create_buffer.hpp"
#include "ax/core/excepts.hpp"

namespace ax::fem {

State::State() = default;

State::State(size_t n_dof_per_vertex, size_t n_vert, BufferDevice device) {
  n_dof_per_vertex_ = n_dof_per_vertex;
  n_vert_ = n_vert;
  device_ = device;

  variables_ = create_buffer<Real>(device, {n_dof_per_vertex, n_vert});
  condition_ = create_buffer<VariableCondition>(device, {n_dof_per_vertex, n_vert});

  variables_->SetBytes(0);  // zero init.
  condition_->SetBytes(0);  // None
}

BufferPtr<VariableCondition> State::GetCondition() const {
  return condition_;
}

BufferPtr<Real> State::GetVariables() const {
  return variables_;
}

void State::SetData(ConstRealBufferView variables, ConstBufferView<VariableCondition> condition) {
  auto device = variables_->Device();
  if (variables.Shape() != variables_->Shape() || condition.Shape() != condition_->Shape()) {
    AX_THROW_RUNTIME_ERROR("Shape mismatch.");
  }

  if (variables.Device() != device || condition.Device() != device) {
    AX_THROW_RUNTIME_ERROR("Device mismatch.");
  }

  copy(variables_->View(), variables);
  copy(condition_->View(), condition);
}

size_t State::GetNumDOFPerVertex() const {
  return n_dof_per_vertex_;
}

size_t State::GetNumVertices() const {
  return n_vert_;
}

}  // namespace ax::fem