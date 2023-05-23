#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "axes/gui/details/scene_pipeline_base.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "axes/core/ecs/ecs.hpp"
#include "axes/core/utils/common.hpp"
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/scene_pass.hpp"
#include "axes/gui/details/vkgraphics.hpp"
namespace axes::gui {

ScenePipelineBase::ScenePipelineBase(std::shared_ptr<VkContext> vkc,
                                     std::weak_ptr<VkGraphicsContext> vkg,
                                     std::weak_ptr<SceneRenderPass> render_pass)
    : vkc_(vkc), vkg_(vkg), render_pass_(render_pass) {}

ScenePipelineBase::~ScenePipelineBase() {
  for (auto ub : uniform_buffers_) {
    vkc_->FreeBuffer(ub);
  }
  uniform_buffers_.clear();
  auto device = vkc_->GetDevice();
  device.destroy(pipeline_layout_);
  device.destroy(pipeline_);
  device.destroy(descriptor_set_layout_);
}

void ScenePipelineBase::CreateUniformBuffer() {
  uint32_t sizeof_ubo = sizeof(details::SceneUniform);
  uniform_buffers_.clear();
  auto vkgl = utils::lock_or_throw(vkg_);
  for (size_t i = 0; i < vkgl->GetSwapchainSize(); ++i) {
    vk::BufferCreateInfo bci;
    bci.setSize(sizeof_ubo).setUsage(vk::BufferUsageFlagBits::eUniformBuffer);
    VmaAllocationCreateInfo alci;
    memset(&alci, 0, sizeof(alci));
    alci.usage = VMA_MEMORY_USAGE_AUTO;
    alci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                 | VMA_ALLOCATION_CREATE_HOST_ACCESS_ALLOW_TRANSFER_INSTEAD_BIT
                 | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    auto buffer = vkc_->AllocateBuffer(bci, alci);
    uniform_buffers_.push_back(buffer);
  }
}

void ScenePipelineBase::CreateDescriptors(vk::DescriptorPool pool) {
  vk::DescriptorSetLayoutBinding ubo_layout_binding;
  ubo_layout_binding.setBinding(0)
      .setDescriptorCount(1)
      .setDescriptorType(vk::DescriptorType::eUniformBuffer)
      .setStageFlags(vk::ShaderStageFlagBits::eVertex
                     | vk::ShaderStageFlagBits::eFragment);

  vk::DescriptorSetLayoutCreateInfo layout_create_info;
  layout_create_info.setBindings(ubo_layout_binding);
  descriptor_set_layout_
      = vkc_->GetDevice().createDescriptorSetLayout(layout_create_info);
  auto vkgl = utils::lock_or_throw(vkg_);
  auto swapchain_size = vkgl->GetSwapchainSize();
  std::vector<vk::DescriptorSetLayout> layouts(swapchain_size,
                                               descriptor_set_layout_);
  vk::DescriptorSetAllocateInfo alloc_info;
  alloc_info.setDescriptorPool(pool).setSetLayouts(layouts).setDescriptorSetCount(
      swapchain_size);
  ubo_descriptor_sets_ = vkc_->GetDevice().allocateDescriptorSets(alloc_info);

  for (size_t i = 0; i < swapchain_size; ++i) {
    vk::DescriptorBufferInfo buffer_info;
    buffer_info.setBuffer(uniform_buffers_[i].buffer_)
        .setOffset(0)
        .setRange(sizeof(details::SceneUniform));
    vk::WriteDescriptorSet desc_write;
    desc_write.setDstSet(ubo_descriptor_sets_[i])
        .setDstBinding(0)
        .setDstArrayElement(0)
        .setDescriptorCount(1)
        .setDescriptorType(vk::DescriptorType::eUniformBuffer)
        .setPBufferInfo(&buffer_info);
    vkc_->GetDevice().updateDescriptorSets(desc_write, {});
  }
}

void ScenePipelineBase::UpdateUniformBuffer() {
  // Obtain the global resources.
  SceneCamera *cam = ecs::Resource<SceneCamera>::MakeValid();
  SceneLight *light = ecs::Resource<SceneLight>::MakeValid();
  SceneProjection *proj = ecs::Resource<SceneProjection>::MakeValid();

  details::SceneUniform ubo;

  // Camera information: View matrix
  ubo.eye_position_ = details::to_glm(cam->position_);
  auto g_pos = ubo.eye_position_;
  auto point = details::to_glm<3>(cam->position_ + cam->front_);
  auto up = details::to_glm<3>(cam->up_);
  ubo.view_ = glm::lookAt(g_pos, point, up);

  // Projection matrix.
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      ubo.projection_[i][j] = proj->projection_(i, j);
    }
  }

  // Light information
  ubo.ambient_light_color_ = details::to_glm(light->ambient_light_color_);
  ubo.parallel_light_color_ = details::to_glm(light->parallel_light_color_);
  ubo.parallel_light_dir_ = details::to_glm(light->parallel_light_dir_);
  ubo.point_light_color_ = details::to_glm(light->point_light_color_);
  ubo.point_light_pos_ = details::to_glm(light->point_light_pos_);

  // Wait for last render finish, and submit the buffer.
  vkc_->GetGraphicsQueue().waitIdle();
  for (auto uf : uniform_buffers_) {
    memcpy(uf.alloc_info_.pMappedData, &ubo, sizeof(details::SceneUniform));
  }
}

VmaAllocationCreateInfo ScenePipelineBase::GetVertexBufferAllocationInfo() {
  VmaAllocationCreateInfo vaci;
  memset(&vaci, 0, sizeof(vaci));
  vaci.usage = VMA_MEMORY_USAGE_AUTO;
  vaci.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
  vaci.priority = 1.0f;
  return vaci;
}

VmaAllocationCreateInfo ScenePipelineBase::GetIndexBufferAllocationInfo() {
  VmaAllocationCreateInfo vaci;
  memset(&vaci, 0, sizeof(vaci));
  vaci.usage = VMA_MEMORY_USAGE_AUTO;
  vaci.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
  vaci.priority = 1.0f;
  return vaci;
}

vk::BufferCreateInfo ScenePipelineBase::GetVertexBufferCreateInfo() {
  vk::BufferCreateInfo bci;
  bci.setUsage(vk::BufferUsageFlagBits::eTransferDst
               | vk::BufferUsageFlagBits::eVertexBuffer)
      .setSharingMode(vk::SharingMode::eExclusive);
  return bci;
}

vk::BufferCreateInfo ScenePipelineBase::GetIndexBufferCreateInfo() {
  vk::BufferCreateInfo bci;
  bci.setUsage(vk::BufferUsageFlagBits::eTransferDst
               | vk::BufferUsageFlagBits::eIndexBuffer)
      .setSharingMode(vk::SharingMode::eExclusive);
  return bci;
}

}  // namespace axes::gui
