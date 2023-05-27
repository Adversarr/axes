#include "axes/gui/details/vkgraphics.hpp"

#include "axes/core/utils/log.hpp"

namespace axes::gui {
static vk::SurfaceFormatKHR choose_swapchain_surface_format(
    std::vector<vk::SurfaceFormatKHR> formats) {
  if (formats.size() == 1 && formats.front() == vk::Format::eUndefined) {
    return vk::SurfaceFormatKHR{vk::Format::eB8G8R8A8Unorm,
                                vk::ColorSpaceKHR::eSrgbNonlinear};
  }

  for (const auto &format : formats) {
    if (format.format == vk::Format::eB8G8R8A8Unorm
        && format.colorSpace == vk::ColorSpaceKHR::eSrgbNonlinear) {
      return format;
    }
  }

  return formats.front();
}

static vk::PresentModeKHR choose_present_mode(
    const std::vector<vk::PresentModeKHR> &modes, bool verbose) {
  auto best = vk::PresentModeKHR::eFifo;
  for (const auto &mode : modes) {
    if (mode == vk::PresentModeKHR::eMailbox) {
      best = mode;
      break;
    }
  }
  if (best != vk::PresentModeKHR::eMailbox) {
    fmt::memory_buffer buffer;
    for (const auto &mode : modes) {
      fmt::format_to(std::back_inserter(buffer), "{} ", vk::to_string(mode));
    }

    if (verbose) {
      AXES_INFO("Not using mailbox present mode, now: {}, available: {}",
                vk::to_string(best), fmt::to_string(buffer));
    }
  } else {
    if (verbose) {
      AXES_INFO("Present Mode: {}", vk::to_string(best));
    }
  }
  return best;
}

static vk::Extent2D choose_extent(std::weak_ptr<GlfwWindow> win,
                                  vk::SurfaceCapabilitiesKHR &capacities) {
  if (capacities.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
    return capacities.currentExtent;
  } else {
    auto window = win.lock();
    if (!window) {
      throw std::runtime_error("Cannot create swapchain when window expired.");
    }
    auto [width, height] = window->GetWindowSize();
    width = std::clamp(width, capacities.minImageExtent.width,
                       capacities.maxImageExtent.height);
    height = std::clamp(height, capacities.minImageExtent.height,
                        capacities.maxImageExtent.height);
    return {width, height};
  }
}

VkGraphicsContext::VkGraphicsContext(std::shared_ptr<VkContext> ctx,
                                     std::shared_ptr<GlfwWindow> win)
    : vk_context_(std::move(ctx)), window_(std::move(win)) {
  CreateCommandPool();
  CreateSwapchain(true);
  CreateImageViews();
  CreateSyncObjects();
}

void VkGraphicsContext::CreateCommandPool() {
  vk::CommandPoolCreateInfo info;
  info.setQueueFamilyIndex(vk_context_->GetSystemInfo()
                               .physical_device_info_.graphics_family_.value())
      .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
  graphics_command_pool_ = vk_context_->GetDevice().createCommandPool(info);
}

void VkGraphicsContext::CreateSwapchain(bool verb) {
  // Find the optimal present mode and image format.
  const auto &support = vk_context_->GetSystemInfo().physical_device_info_;
  vk::SurfaceCapabilitiesKHR surface_capabilities
      = vk_context_->GetPhysicalDevice().getSurfaceCapabilitiesKHR(
          vk_context_->GetSurface());

  auto format = choose_swapchain_surface_format(support.surface_formats_);
  auto present_mode = choose_present_mode(support.surface_present_modes_, verb);
  auto extent = choose_extent(window_, surface_capabilities);
  if (verb) {
    AXES_INFO("Extent: {}x{}", extent.width, extent.height);
    AXES_INFO("Present mode: {}", vk::to_string(present_mode));
    AXES_INFO("Format: {}, Colorspace: {}", vk::to_string(format.format),
              vk::to_string(format.colorSpace));
  }

  auto max_sc_size = surface_capabilities.maxImageCount;
  if (swapchain_size_ > max_sc_size && max_sc_size != 0) {
    AXES_WARN("Required Swapchain size is greater than max supported, {} > {}",
              swapchain_size_, max_sc_size);
    swapchain_size_ = max_sc_size;
  }

  // Create Swapchain.
  vk::SwapchainCreateInfoKHR info;
  info.setSurface(vk_context_->GetSurface())
      .setMinImageCount(swapchain_size_)
      .setImageFormat(format.format)
      .setImageColorSpace(format.colorSpace)
      .setImageExtent(extent)
      .setImageArrayLayers(1)
      .setImageUsage(vk::ImageUsageFlagBits::eColorAttachment);
  auto queue_family_indices = std::array<uint32_t, 2>{
      support.graphics_family_.value(), support.present_family_.value()};
  if (queue_family_indices[0] != queue_family_indices[1]) {
    info.setImageSharingMode(vk::SharingMode::eConcurrent)
        .setQueueFamilyIndices(queue_family_indices);
  } else {
    info.setImageSharingMode(vk::SharingMode::eExclusive);
  }

  info.setPreTransform(surface_capabilities.currentTransform)
      .setCompositeAlpha(vk::CompositeAlphaFlagBitsKHR::eOpaque)
      .setPresentMode(present_mode)
      .setClipped(VK_TRUE)
      .setOldSwapchain(VK_NULL_HANDLE);

  auto device = vk_context_->GetDevice();

  // Create the swapchain.
  swapchain_ = device.createSwapchainKHR(info);
  auto s_imgs = device.getSwapchainImagesKHR(swapchain_);

  swapchain_images_.clear();
  for (auto img : s_imgs) {
    details::VkImageWithView iwv;
    iwv.image_ = img;
    swapchain_images_.push_back(iwv);
  }
  swapchain_image_format_ = format.format;
  swapchain_extent_ = extent;
}
void VkGraphicsContext::CreateImageViews() {
  for (auto &img_view : swapchain_images_) {
    img_view.view_ = CreateImageView(img_view.image_, swapchain_image_format_,
                                     vk::ImageAspectFlagBits::eColor);
  }
}

vk::ImageView VkGraphicsContext::CreateImageView(vk::Image image, vk::Format format,
                                                 vk::ImageAspectFlags aspect) {
  vk::ImageViewCreateInfo view_info{};
  view_info.image = image;
  view_info.viewType = vk::ImageViewType::e2D;
  view_info.format = format;
  view_info.components.r = vk::ComponentSwizzle::eIdentity;
  view_info.components.g = vk::ComponentSwizzle::eIdentity;
  view_info.components.b = vk::ComponentSwizzle::eIdentity;
  view_info.components.a = vk::ComponentSwizzle::eIdentity;
  view_info.subresourceRange.aspectMask = aspect;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  return vk_context_->GetDevice().createImageView(view_info);
}

void VkGraphicsContext::CreateSyncObjects() {
  vk::SemaphoreCreateInfo semaphore_info;
  vk::FenceCreateInfo fence_info;
  fence_info.flags = vk::FenceCreateFlagBits::eSignaled;
  auto device = vk_context_->GetDevice();
  for (size_t i = 0; i < swapchain_size_; ++i) {
    details::RenderSyncObjects sync_obj;
    sync_obj.render_finished_ = device.createSemaphore(semaphore_info);
    sync_obj.image_available_ = device.createSemaphore(semaphore_info);
    sync_obj.in_flight_fence_ = device.createFence(fence_info);
    syncs_.push_back(sync_obj);
  }
}

VkGraphicsContext::~VkGraphicsContext() {
  auto device = vk_context_->GetDevice();
  device.waitIdle();
  DestroySwapchain();
  for (auto &sync : syncs_) {
    device.destroy(sync.image_available_);
    device.destroy(sync.render_finished_);
    device.destroy(sync.in_flight_fence_);
  }
  syncs_.clear();
  device.destroyCommandPool(graphics_command_pool_);
}

void VkGraphicsContext::DestroySwapchain() {
  auto device = vk_context_->GetDevice();
  for (auto &view : swapchain_images_) {
    device.destroy(view.view_);
  }
  device.destroy(swapchain_);
}

VkRenderResult VkGraphicsContext::BeginRender() {
  auto device = vk_context_->GetDevice();
  // Timeout = infinity.
  AXES_CHECK(device.waitForFences(syncs_[current_frame_].in_flight_fence_, VK_TRUE,
                                  std::numeric_limits<uint64_t>::max())
                 == vk::Result::eSuccess,
             "Failed to wait for fence.");

  // Acquire Image
  auto rv
      = device.acquireNextImageKHR(swapchain_, std::numeric_limits<uint64_t>::max(),
                                   syncs_[current_frame_].image_available_);

  if (rv.result == vk::Result::eErrorOutOfDateKHR) {
    return VkRenderResult::kRecreateSwapchain;
  }
  // Check acquire the image successfully.
  AXES_CHECK(
      rv.result == vk::Result::eSuccess || rv.result == vk::Result::eSuboptimalKHR,
      "Acquire next image failed, with result = {}", vk::to_string(rv.result));
  current_image_index_ = rv.value;
  device.resetFences(syncs_[current_frame_].in_flight_fence_);
  draw_started_ = true;
  return VkRenderResult::kSuccess;
}

VkRenderResult VkGraphicsContext::EndRender(std::vector<vk::CommandBuffer> cbufs) {
  vk::SubmitInfo submit_info{};
  auto wait_sem = std::array{syncs_[current_frame_].image_available_};
  auto signal_sem = std::array{syncs_[current_frame_].render_finished_};
  auto wait_stages = std::array<vk::PipelineStageFlags, 1>{
      vk::PipelineStageFlagBits::eColorAttachmentOutput};
  submit_info.setWaitSemaphores(wait_sem)
      .setPWaitDstStageMask(wait_stages.data())
      .setCommandBuffers(cbufs)
      .setSignalSemaphores(signal_sem);
  vk_context_->GetGraphicsQueue().submit(submit_info,
                                         syncs_[current_frame_].in_flight_fence_);

  vk::PresentInfoKHR present_info;
  auto render_finish = std::array{syncs_[current_frame_].render_finished_};
  present_info.setWaitSemaphores(render_finish)
      .setSwapchains(swapchain_)
      .setPImageIndices(&current_image_index_);

  auto result = vk_context_->GetPresentQueue().presentKHR(&present_info);
  bool need_recreate_swapchain = false;
  if (result == vk::Result::eErrorOutOfDateKHR
      || result == vk::Result::eSuboptimalKHR || window_->IsResized()) {
    AXES_DEBUG_LOG("Set Recreate Swapchain. result is {}", vk::to_string(result));
    window_->ResetResizeFlag();
    need_recreate_swapchain = true;
  } else {
    AXES_CHECK(result == vk::Result::eSuccess, "Failed to present swapchain image");
  }
  current_frame_ = (current_frame_ + 1) % swapchain_size_;
  draw_started_ = false;
  return need_recreate_swapchain ? VkRenderResult::kRecreateSwapchain
                                 : VkRenderResult::kSuccess;
}

VkRenderResult VkGraphicsContext::Render() {
  auto r = BeginRender();
  if (r != VkRenderResult::kSuccess) {
    return r;
  }

  std::vector<vk::CommandBuffer> cbufs;
  cbufs.reserve(render_passes_.size());
  for (const auto &rp : render_passes_) {
    cbufs.push_back(rp->Draw());
  }
  r = EndRender(cbufs);
  return r;
}

void VkGraphicsContext::RecreateSwapchain() {
  window_->UpdateWindowSize();
  vk_context_->GetDevice().waitIdle();
  DestroySwapchain();
  CreateSwapchain(false);
  CreateImageViews();

  // acknowledge passes
  for (const auto &passes : render_passes_) {
    passes->RecreateSwapchain();
  }
}

const std::vector<details::VkImageWithView> &
VkGraphicsContext::GetSwapchainImageWithView() const noexcept {
  return swapchain_images_;
}

uint32_t VkGraphicsContext::GetAcquiredImageIndex() const noexcept {
  return current_image_index_;
}

uint32_t VkGraphicsContext::GetFrameIndex() const noexcept {
  return current_frame_;
}

std::weak_ptr<GlfwWindow> VkGraphicsContext::GetWindow() { return window_; }

vk::CommandPool VkGraphicsContext::GetCommandPool() {
  return graphics_command_pool_;
}

uint32_t VkGraphicsContext::GetSwapchainSize() const noexcept {
  return swapchain_size_;
}

void VkGraphicsContext::RegisterRenderPass(std::shared_ptr<RenderPassBase> p) {
  render_passes_.push_back(p);
}

vk::Format VkGraphicsContext::GetSwapchainFormat() const noexcept {
  return swapchain_image_format_;
}

vk::Extent2D VkGraphicsContext::GetSwapchainExtent() const noexcept {
  return swapchain_extent_;
}

}  // namespace axes::gui
