#pragma once
#include <vector>
#include <vulkan/vulkan.hpp>

#include "axes/core/ecs/ecs.hpp"  // IWYU pragma: export
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/common.hpp"
#include "axes/gui/details/staging_buffer.hpp"
#include "axes/gui/details/vkcontext.hpp"
namespace axes::gui {
class ScenePipelineBase {
public:
  ScenePipelineBase(std::shared_ptr<VkContext> vkc,
                    std::weak_ptr<VkGraphicsContext> vkg,
                    std::weak_ptr<SceneRenderPass> render_pass);

  virtual void CreateEntityRenderData(ecs::EntityID ent, StagingBuffer& sbuffer) = 0;

  /**
   * @brief Draw call for the pipeline.
   *
   * @param buffer
   */
  virtual void Draw(vk::CommandBuffer& buffer) = 0;

  virtual ~ScenePipelineBase();

  /**
   * @brief Update from global resources
   *
   */
  virtual void UpdateUniformBuffer();

protected:
  virtual void CreatePipeline(vk::RenderPass pass) = 0;

  virtual void CreateUniformBuffer();

  virtual void CreateDescriptors(vk::DescriptorPool uniform_pool);

  vk::BufferCreateInfo GetVertexBufferCreateInfo();
  vk::BufferCreateInfo GetIndexBufferCreateInfo();
  VmaAllocationCreateInfo GetVertexBufferAllocationInfo();
  VmaAllocationCreateInfo GetIndexBufferAllocationInfo();

  std::vector<vk::DescriptorSet> ubo_descriptor_sets_;
  vk::DescriptorSetLayout descriptor_set_layout_;
  vk::PipelineLayout pipeline_layout_;
  vk::Pipeline pipeline_;
  std::vector<VmaAllocBuffer> uniform_buffers_;

  std::shared_ptr<VkContext> vkc_;
  std::weak_ptr<VkGraphicsContext> vkg_;
  std::weak_ptr<SceneRenderPass> render_pass_;
};
}  // namespace axes::gui
