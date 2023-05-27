#include "axes/gui/details/ui_pass.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <axes/gui/imnodes/imnodes.hpp>

#include "axes/core/ecs/resource_manager.hpp"
#include "axes/core/utils/log.hpp"
#include "axes/gui/details/scene_data.hpp"
#include "axes/gui/details/vkgraphics.hpp"

static void check_vk_result(VkResult err) {
  AXES_CHECK(err == VK_SUCCESS, "error detected: {}",
             vk::to_string(static_cast<vk::Result>(err)));
}

namespace axes::gui {

UiRenderPass::UiRenderPass(std::shared_ptr<VkContext> vkc,
                           std::weak_ptr<VkGraphicsContext> vkg)
    : vkc_(vkc), vkg_(vkg) {
  auto vkg_l = vkg_.lock();
  if (!vkg_l) {
    throw std::runtime_error("Cannot initialize when Graphics Context expired");
  }
  window_ = vkg_l->GetWindow();
  auto win_l = window_.lock();

  vk::DescriptorPoolSize pool_sizes[]
      = {{vk::DescriptorType::eSampler, 1000},
         {vk::DescriptorType::eCombinedImageSampler, 1000},
         {vk::DescriptorType::eSampledImage, 1000},
         {vk::DescriptorType::eStorageImage, 1000},
         {vk::DescriptorType::eUniformTexelBuffer, 1000},
         {vk::DescriptorType::eStorageTexelBuffer, 1000},
         {vk::DescriptorType::eUniformBuffer, 1000},
         {vk::DescriptorType::eStorageBuffer, 1000},
         {vk::DescriptorType::eUniformBufferDynamic, 1000},
         {vk::DescriptorType::eStorageBufferDynamic, 1000},
         {vk::DescriptorType::eInputAttachment, 1000}};

  vk::DescriptorPoolCreateInfo pool_info = {};
  pool_info.setFlags(vk::DescriptorPoolCreateFlagBits::eFreeDescriptorSet)
      .setMaxSets(1000 * IM_ARRAYSIZE(pool_sizes))
      .setPoolSizes(pool_sizes);
  descriptor_pool_ = vkc_->GetDevice().createDescriptorPool(pool_info);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImNodes::CreateContext();
  ImGui_ImplGlfw_InitForVulkan(win_l->GetWindow(), true);

  auto device = vkc_->GetDevice();
  auto info = vkc_->GetSystemInfo();
  {  // Command Pool and Command Buffer.
    vk::CommandPoolCreateInfo pool_info;
    pool_info
        .setQueueFamilyIndex(
            vkc_->GetSystemInfo().physical_device_info_.graphics_family_.value())
        .setFlags(vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
    command_pool_ = device.createCommandPool(pool_info);

    vk::CommandBufferAllocateInfo info;
    info.setCommandPool(command_pool_)
        .setCommandBufferCount(vkg_l->GetSwapchainSize())
        .setLevel(vk::CommandBufferLevel::ePrimary);
    command_buffers_ = device.allocateCommandBuffers(info);
  }

  {  // Render Pass
    vk::AttachmentDescription attachment = {};
    attachment.setFormat(vkg_l->GetSwapchainFormat())
        .setSamples(vk::SampleCountFlagBits::e1)
        .setStoreOp(vk::AttachmentStoreOp::eStore)
        .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
        .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
        .setFinalLayout(vk::ImageLayout::ePresentSrcKHR);
    // Assert the UI pass is the final pass.
    attachment.setLoadOp(vk::AttachmentLoadOp::eLoad)
        .setInitialLayout(vk::ImageLayout::eColorAttachmentOptimal);

    vk::AttachmentReference color_attachment = {};
    color_attachment.setAttachment(0).setLayout(
        vk::ImageLayout::eColorAttachmentOptimal);
    vk::SubpassDescription subpass;
    subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
        .setColorAttachments(color_attachment);
    vk::SubpassDependency subpass_dep;
    subpass_dep.setSrcSubpass(VK_SUBPASS_EXTERNAL)
        .setDstSubpass(0)
        .setSrcStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput)
        .setDstStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput)
        .setSrcAccessMask(vk::AccessFlagBits::eColorAttachmentWrite)
        .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite);
    vk::RenderPassCreateInfo info;
    info.setAttachments(attachment)
        .setSubpasses(subpass)
        .setDependencies(subpass_dep);
    render_pass_ = device.createRenderPass(info);
  }

  // this initializes imgui for Vulkan
  ImGui_ImplVulkan_InitInfo init_info = {};
  init_info.Instance = vkc_->GetInstance();
  init_info.PhysicalDevice = vkc_->GetPhysicalDevice();
  init_info.Device = device;
  init_info.Queue = vkc_->GetGraphicsQueue();
  init_info.QueueFamily = info.physical_device_info_.graphics_family_.value();
  init_info.Allocator = VK_NULL_HANDLE;
  init_info.DescriptorPool = descriptor_pool_;
  init_info.MinImageCount = vkg_l->GetSwapchainSize();
  init_info.ImageCount = vkg_l->GetSwapchainSize();
  init_info.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
  init_info.CheckVkResultFn = check_vk_result;

  AXES_CHECK(ImGui_ImplVulkan_Init(&init_info, render_pass_),
             "Falied to init ImGui_Vulkan");

  {  // Upload Fonts and textures
    vkc_->RunTransientCommandInplace(
        [](vk::CommandBuffer cbuf) { ImGui_ImplVulkan_CreateFontsTexture(cbuf); },
        vkc_->GetGraphicsQueue());
    ImGui_ImplVulkan_DestroyFontUploadObjects();
  }

  CreateFramebuffers();
}

UiRenderPass::~UiRenderPass() {
  DestroyFramebuffers();
  auto device = vkc_->GetDevice();
  device.destroy(render_pass_);
  device.destroy(descriptor_pool_);
  device.destroy(command_pool_);
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImNodes::DestroyContext();
  ImGui::DestroyContext();
}

void UiRenderPass::CreateFramebuffers() {
  auto vkg_l = vkg_.lock();
  if (!vkg_l) {
    throw std::runtime_error("Graphics Context Unavailable.");
  }

  for (auto imageview : vkg_l->GetSwapchainImageWithView()) {
    auto attachments = std::array{imageview.view_};
    auto extent = vkg_l->GetSwapchainExtent();
    vk::FramebufferCreateInfo info;
    info.setRenderPass(render_pass_)
        .setAttachments(attachments)
        .setWidth(extent.width)
        .setHeight(extent.height)
        .setLayers(1);
    framebuffers_.emplace_back(vkc_->GetDevice().createFramebuffer(info));
  }
}

void UiRenderPass::DestroyFramebuffers() {
  auto device = vkc_->GetDevice();
  for (auto fb : framebuffers_) {
    device.destroy(fb);
  }
  framebuffers_.clear();
}

void UiRenderPass::RecreateSwapchain() {
  DestroyFramebuffers();
  CreateFramebuffers();
}

vk::CommandBuffer UiRenderPass::Draw() {
  auto vkg_l = vkg_.lock();
  if (!vkg_l) {
    throw std::runtime_error("Cannot initialize when Graphics Context expired");
  }
  size_t current_index = vkg_l->GetFrameIndex();
  size_t current_image_index = vkg_l->GetAcquiredImageIndex();
  auto& current_command_buffer = command_buffers_[current_index];
  // Begin Command Buffer.
  vk::CommandBufferBeginInfo begininfo;
  current_command_buffer.begin(begininfo);
  // Begin Render pass.
  vk::RenderPassBeginInfo render_pass_begin_info;
  vk::ClearColorValue empty_color{std::array{0.0f, 0.0f, 0.0f, 1.0f}};
  vk::ClearValue clear_values[] = {empty_color};
  render_pass_begin_info.setFramebuffer(framebuffers_[current_image_index])
      .setRenderPass(render_pass_)
      .setClearValueCount(1)
      .setPClearValues(clear_values);
  render_pass_begin_info.renderArea.setExtent(vkg_l->GetSwapchainExtent());
  render_pass_begin_info.renderArea.setOffset({0, 0});
  current_command_buffer.beginRenderPass(render_pass_begin_info,
                                         vk::SubpassContents::eInline);
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  DrawUI();
  ImGui::Render();
  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), current_command_buffer);
  // End render pass and end command buffer.
  current_command_buffer.endRenderPass();
  current_command_buffer.end();
  return current_command_buffer;
}

void UiRenderPass::DrawUI() {
  auto win = ecs::Rc<UiWindows>{}.MakeValid();
  if (win->menu_bar_) {
    win->menu_bar_();
  }
  for (auto& f : win->callbacks_) {
    f();
  }
}

vk::DescriptorPool UiRenderPass::GetDescriptorPool() { return descriptor_pool_; }

}  // namespace axes::gui
