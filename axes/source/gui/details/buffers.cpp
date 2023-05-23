#include "axes/gui/details/buffers.hpp"

#include <axes/core/ecs/ecs.hpp>
namespace axes::gui::details {

std::vector<vk::VertexInputBindingDescription>
SceneVertex::GetBindingDescriptions() {
  vk::VertexInputBindingDescription binding;
  binding.setBinding(0);
  binding.setStride(sizeof(SceneVertex));
  binding.setInputRate(vk::VertexInputRate::eVertex);
  return {binding};
}

std::vector<vk::VertexInputAttributeDescription>
SceneVertex::GetAttributeDescriptions() {
  vk::VertexInputAttributeDescription desc1;
  desc1.binding = 0;
  desc1.location = 0;
  desc1.format = vk::Format::eR32G32B32Sfloat;
  desc1.offset = offsetof(SceneVertex, position_);

  vk::VertexInputAttributeDescription desc2;
  desc2.binding = 0;
  desc2.location = 1;
  desc2.format = vk::Format::eR32G32B32A32Sfloat;
  desc2.offset = offsetof(SceneVertex, color_);

  vk::VertexInputAttributeDescription desc3;
  desc3.binding = 0;
  desc3.location = 2;
  desc3.format = vk::Format::eR32G32B32Sfloat;
  desc3.offset = offsetof(SceneVertex, normal_);
  return {desc1, desc2, desc3};
}

std::vector<vk::VertexInputBindingDescription>
MeshInstance::GetBindingDescriptions() {
  vk::VertexInputBindingDescription binding;
  binding.setBinding(1);
  binding.setStride(sizeof(MeshInstance));
  binding.setInputRate(vk::VertexInputRate::eInstance);
  return {binding};
}

std::vector<vk::VertexInputAttributeDescription>
MeshInstance::GetAttributeDescriptions() {
  vk::VertexInputAttributeDescription desc1;
  desc1.binding = 1;
  desc1.location = 4;
  desc1.format = vk::Format::eR32G32B32Sfloat;
  desc1.offset = offsetof(MeshInstance, position_);

  vk::VertexInputAttributeDescription desc2;
  desc2.binding = 1;
  desc2.location = 5;
  desc2.format = vk::Format::eR32G32B32A32Sfloat;
  desc2.offset = offsetof(MeshInstance, rotation_);

  vk::VertexInputAttributeDescription desc3;
  desc3.binding = 1;
  desc3.location = 6;
  desc3.format = vk::Format::eR32G32B32A32Sfloat;
  desc3.offset = offsetof(MeshInstance, color_);
  return {desc1, desc2, desc3};
}

MeshRenderDataGpu::~MeshRenderDataGpu() {
  if (index_buffer_.buffer_) {
    vkc_->FreeBuffer(index_buffer_);
  }

  if (instance_buffer_.buffer_) {
    vkc_->FreeBuffer(instance_buffer_);
  }
}

SceneRenderDataSharedGpu::~SceneRenderDataSharedGpu() {
  if (vertex_buffer_.buffer_) {
    vkc_->FreeBuffer(vertex_buffer_);
  }
}

}  // namespace axes::gui::details
