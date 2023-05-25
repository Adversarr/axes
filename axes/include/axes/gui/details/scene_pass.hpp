#pragma once

#include "axes/gui/details/vkgraphics.hpp"
#include "render_pass_base.hpp"
#include <glm/fwd.hpp>
#include <iostream>

namespace axes::gui {

struct SceneCamera {
  RealVector3 position_;
  RealVector3 up_;
  RealVector3 front_;
  static void InitResource();
};

struct SceneLight {
  RealVector3 point_light_pos_;
  RealVector4 point_light_color_;
  RealVector3 parallel_light_dir_;
  RealVector4 parallel_light_color_;
  RealVector4 ambient_light_color_;
  static void InitResource();
};

enum class ProjectionMode: int {
  kPerspective,
  kOrtho
};

struct SceneProjection {
  RealMat4x4 projection_;
  ProjectionMode mode_;
  static void InitResource();
};

struct SceneOrthoProjection {
  Real left_, right_, bottom_, top_, near_, far_;
  static void InitResource();
};

struct ScenePerspectiveProjection {
  Real wh_ratio_, near_, far_, fovy_;
  static void InitResource();
};

RealMat4x4 compute_perspective(ScenePerspectiveProjection proj);

/**
 * @brief Major render pass of this library.
 * 
 */
class SceneRenderPass : public RenderPassBase {
public:
  explicit SceneRenderPass(std::shared_ptr<VkContext> vkc,
                           std::weak_ptr<VkGraphicsContext> vkg);

  /**
   * @brief Recreate Swapchain.
   * 
   */
  void RecreateSwapchain() final;

  /**
   * @brief Get the draw-call command buffer.
   * 
   * @return vk::CommandBuffer 
   */
  vk::CommandBuffer Draw() final;

  /**
   * @brief Get the (uniform) Descriptor Pool
   * 
   * @return vk::DescriptorPool 
   */
  vk::DescriptorPool GetDescriptorPool() final;

  /**
   * @brief Add a pipeline for the scene.
   * 
   * @param pipeline 
   */
  void AddPipeline(std::shared_ptr<ScenePipelineBase> pipeline);

  ~SceneRenderPass();

  vk::RenderPass GetRenderPass();

private:
  void CreateFramebuffers();
  void DestroySwapchain();
  void CreateRenderPass();
  void CreateDepthResources();
  void CreateCommandBuffers();
  void CreateDescriptorPool();

  vk::Format FindDepthFormat();

  std::vector<std::shared_ptr<ScenePipelineBase>> pipelines_;

  vk::ClearColorValue background_color_{std::array{0.05f, 0.05f, 0.05f, 1.0f}};
  vk::ClearDepthStencilValue depth_stencil_value_{{1.0f, 0}};
  vk::CommandPool command_pool_;
  std::vector<vk::CommandBuffer> command_buffers_;
  vk::RenderPass render_pass_;
  vk::DescriptorPool uniform_descriptor_pool_;
  std::vector<vk::Framebuffer> framebuffers_;

  // Depth buffers
  VmaAllocImage depth_image_;
  vk::ImageView depth_image_view_;


  // Basic information
  std::shared_ptr<VkContext> vkc_;
  std::weak_ptr<VkGraphicsContext> vkg_;
  std::weak_ptr<GlfwWindow> window_;
};
} // namespace axes::gui
