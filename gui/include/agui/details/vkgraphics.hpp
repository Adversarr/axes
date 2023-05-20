#pragma once
#include <vma/vk_mem_alloc.h>

#include "agui/details/common.hpp"
#include "agui/details/render_pass_base.hpp"
#include "agui/details/vkcontext.hpp"

namespace axes::gui {

namespace details {

struct VkImageWithView {
  vk::Image image_;
  vk::ImageView view_;
};

struct RenderSyncObjects {
  vk::Semaphore image_available_;
  vk::Semaphore render_finished_;
  vk::Fence in_flight_fence_;
};

}  // namespace details

enum class VkRenderResult {
  kSuccess,
  kRecreateSwapchain,
  kFailed,
};

class VkGraphicsContext {
public:
  VkGraphicsContext(std::shared_ptr<VkContext> ctx,
                    std::shared_ptr<GlfwWindow> win);

  ~VkGraphicsContext();

  void RegisterRenderPass(std::shared_ptr<RenderPassBase> p) {
    render_passes_.push_back(p);
  }

  VkRenderResult Render();

  uint32_t GetSwapchainSize() const noexcept { return swapchain_size_; }

  vk::Format GetSwapchainFormat() const noexcept { return swapchain_image_format_; }

  vk::Extent2D GetSwapchainExtent() const noexcept { return swapchain_extent_; }

  const std::vector<details::VkImageWithView>& GetSwapchainImageWithView()
      const noexcept {
    return swapchain_images_;
  }

  uint32_t GetAcquiredImageIndex() const noexcept {
    return current_image_index_;
  }

  uint32_t GetFrameIndex() const noexcept { return current_frame_; }

  std::weak_ptr<GlfwWindow> GetWindow() { return window_; }

  vk::CommandPool GetCommandPool() { return graphics_command_pool_; }

  void RecreateSwapchain();
private:
  VkRenderResult BeginRender();
  VkRenderResult EndRender(std::vector<vk::CommandBuffer> cbufs);
  void CreateCommandPool();
  vk::ImageView CreateImageView(vk::Image image, vk::Format format,
                                vk::ImageAspectFlags aspectFlags);
  void CreateImageViews();
  void CreateSyncObjects();
  void CreateSwapchain(bool verb);
  void DestroySwapchain();

  std::shared_ptr<VkContext> vk_context_;
  std::shared_ptr<GlfwWindow> window_;

  vk::Queue graphics_queue_;
  vk::Queue present_queue_;
  std::vector<std::shared_ptr<RenderPassBase>> render_passes_;
  // Swapchain
  vk::SwapchainKHR swapchain_;
  vk::Format swapchain_image_format_;
  vk::Extent2D swapchain_extent_;
  vk::CommandPool graphics_command_pool_;

  std::vector<details::VkImageWithView> swapchain_images_;

  std::vector<details::RenderSyncObjects> syncs_;

  size_t current_frame_{0};
  uint32_t swapchain_size_{3};
  bool draw_started_{false};
  uint32_t current_image_index_{0};
};

}  // namespace axes::gui
