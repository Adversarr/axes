#pragma once

#include "axes/gui/details/vkcontext.hpp"
#include "render_pass_base.hpp"

namespace axes::gui {

/**
 * @class UiRenderPass
 * @brief The ImGUI Render Pass.
 *
 * @warn This pass should, and must be the last pass.
 *
 */
class UiRenderPass : public RenderPassBase {
public:
  explicit UiRenderPass(std::shared_ptr<VkContext> vkc,
                        std::weak_ptr<VkGraphicsContext> vkg);

  vk::DescriptorPool GetDescriptorPool() final;

  void RecreateSwapchain() final;

  vk::CommandBuffer Draw() final;

  /**
   * @brief Poly. Func. for ImGui window management.
   */
  virtual void DrawUI();

  ~UiRenderPass();

private:
  void CreateFramebuffers();
  void DestroyFramebuffers();
  vk::CommandPool command_pool_;
  std::vector<vk::CommandBuffer> command_buffers_;
  vk::RenderPass render_pass_;
  vk::DescriptorPool descriptor_pool_;
  std::vector<vk::Framebuffer> framebuffers_;
  std::shared_ptr<VkContext> vkc_;
  std::weak_ptr<VkGraphicsContext> vkg_;
  std::weak_ptr<GlfwWindow> window_;
};

}  // namespace axes::gui
