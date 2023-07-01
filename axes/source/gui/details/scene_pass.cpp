#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include "axes/gui/details/scene_pass.hpp"

#include <Eigen/Geometry>
#include <axes/core/ecs/ecs.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/scene_pipeline_base.hpp"

namespace axes::gui {
SceneRenderPass::SceneRenderPass(std::shared_ptr<VkContext> vkc,
                                 std::weak_ptr<VkGraphicsContext> vkg)
    : vkc_(std::move(vkc)), vkg_(std::move(vkg)) {
  if (auto vkgl = vkg_.lock(); vkgl) {
    window_ = vkgl->GetWindow();
  } else {
    throw std::runtime_error("Failed to lock VkGraphicsContext");
  }

  CreateRenderPass();
  CreateDepthResources();
  CreateFramebuffers();
  CreateCommandBuffers();
  CreateDescriptorPool();
}

vk::Format SceneRenderPass::FindDepthFormat() {
  auto physical_device = vkc_->GetPhysicalDevice();
  auto tiling = vk::ImageTiling::eOptimal;
  vk::FormatFeatureFlags features
      = vk::FormatFeatureFlagBits::eDepthStencilAttachment;

  for (const auto& format : {vk::Format::eD32Sfloat, vk::Format::eD32SfloatS8Uint,
                             vk::Format::eD24UnormS8Uint}) {
    auto props = physical_device.getFormatProperties(format);

    if (tiling == vk::ImageTiling::eLinear
        && (props.linearTilingFeatures & features) == features) {
      return format;
    } else if (tiling == vk::ImageTiling::eOptimal
               && (props.optimalTilingFeatures & features) == features) {
      return format;
    }
  }
  throw std::runtime_error("Failed to find a suitable depth format");
}

void SceneRenderPass::CreateRenderPass() {
  vk::AttachmentDescription color_attachment;
  auto vkgl = vkg_.lock();
  if (!vkgl) {
    throw std::runtime_error("Cannot create render pass when vkg cannot lock.");
  }
  auto sw_img_fmt = vkgl->GetSwapchainFormat();
  color_attachment.setFormat(sw_img_fmt)
      .setSamples(vk::SampleCountFlagBits::e1)
      .setLoadOp(vk::AttachmentLoadOp::eClear)
      .setStoreOp(vk::AttachmentStoreOp::eStore)
      .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
      .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
      .setInitialLayout(vk::ImageLayout::eUndefined)
      .setFinalLayout(vk::ImageLayout::eColorAttachmentOptimal);
  color_attachment.setFinalLayout(vk::ImageLayout::eColorAttachmentOptimal);
  vk::AttachmentReference color_attachment_ref;
  color_attachment_ref.setAttachment(0).setLayout(
      vk::ImageLayout::eColorAttachmentOptimal);

  vk::AttachmentDescription depth_attachment;
  depth_attachment.setFormat(FindDepthFormat())
      .setSamples(vk::SampleCountFlagBits::e1)
      .setLoadOp(vk::AttachmentLoadOp::eClear)
      .setStoreOp(vk::AttachmentStoreOp::eStore)
      .setStencilStoreOp(vk::AttachmentStoreOp::eDontCare)
      .setStencilLoadOp(vk::AttachmentLoadOp::eDontCare)
      .setInitialLayout(vk::ImageLayout::eUndefined)
      .setFinalLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal);

  vk::AttachmentReference depth_ref;
  depth_ref.setLayout(vk::ImageLayout::eDepthStencilAttachmentOptimal)
      .setAttachment(1);

  vk::SubpassDescription subpass;
  subpass.setPipelineBindPoint(vk::PipelineBindPoint::eGraphics)
      .setColorAttachmentCount(1)
      .setPDepthStencilAttachment(&depth_ref)
      .setPColorAttachments(&color_attachment_ref);

  vk::SubpassDependency dependency;
  dependency.setSrcSubpass(VK_SUBPASS_EXTERNAL)
      .setDstSubpass(0)
      .setSrcStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput
                       | vk::PipelineStageFlagBits::eEarlyFragmentTests)
      .setSrcAccessMask(vk::AccessFlagBits::eNone)
      .setDstStageMask(vk::PipelineStageFlagBits::eColorAttachmentOutput
                       | vk::PipelineStageFlagBits::eEarlyFragmentTests)
      .setDstAccessMask(vk::AccessFlagBits::eColorAttachmentWrite
                        | vk::AccessFlagBits::eDepthStencilAttachmentWrite);

  vk::RenderPassCreateInfo info;
  auto attachments = std::array{color_attachment, depth_attachment};
  info.setAttachments(attachments)
      .setSubpasses(subpass)
      .setDependencies(dependency);
  render_pass_ = vkc_->GetDevice().createRenderPass(info);
}

void SceneRenderPass::CreateDepthResources() {
  auto depth_format = FindDepthFormat();
  auto vkgl = vkg_.lock();
  if (!vkgl) {
    throw std::runtime_error("Failed to lock vkg, cannot create depth resource");
  }
  VkExtent2D extent = vkgl->GetSwapchainExtent();
  vk::ImageCreateInfo image_info{};
  image_info.imageType = vk::ImageType::e2D;
  image_info.extent.width = extent.width;
  image_info.extent.height = extent.height;
  image_info.extent.depth = 1;
  image_info.mipLevels = 1;
  image_info.arrayLayers = 1;
  image_info.format = depth_format;
  image_info.tiling = vk::ImageTiling::eOptimal;
  image_info.initialLayout = vk::ImageLayout::eUndefined;
  image_info.usage = vk::ImageUsageFlagBits::eDepthStencilAttachment;
  image_info.samples = vk::SampleCountFlagBits::e1;
  image_info.sharingMode = vk::SharingMode::eExclusive;

  VmaAllocationCreateInfo al_info;
  memset(&al_info, 0, sizeof(al_info));
  al_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  al_info.preferredFlags = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;
  depth_image_ = vkc_->AllocateImage(image_info, al_info);

  vk::ImageViewCreateInfo view_info{};
  view_info.image = depth_image_.image_;
  view_info.viewType = vk::ImageViewType::e2D;
  view_info.format = depth_format;
  view_info.components.r = vk::ComponentSwizzle::eIdentity;
  view_info.components.g = vk::ComponentSwizzle::eIdentity;
  view_info.components.b = vk::ComponentSwizzle::eIdentity;
  view_info.components.a = vk::ComponentSwizzle::eIdentity;
  view_info.subresourceRange.aspectMask = vk::ImageAspectFlagBits::eDepth;
  view_info.subresourceRange.baseMipLevel = 0;
  view_info.subresourceRange.levelCount = 1;
  view_info.subresourceRange.baseArrayLayer = 0;
  view_info.subresourceRange.layerCount = 1;
  depth_image_view_ = vkc_->GetDevice().createImageView(view_info);
}

void SceneRenderPass::CreateFramebuffers() {
  auto vkgl = vkg_.lock();
  if (!vkgl) {
    throw std::runtime_error("Failed to lock vkg, cannot create depth resource");
  }
  VkExtent2D extent = vkgl->GetSwapchainExtent();
  if (!framebuffers_.empty()) {
    throw std::logic_error("Framebuffer is not empty when creation.");
  }
  for (auto imageview : vkgl->GetSwapchainImageWithView()) {
    auto attachments = std::array{imageview.view_, depth_image_view_};
    vk::FramebufferCreateInfo info;
    info.setRenderPass(render_pass_)
        .setAttachments(attachments)
        .setWidth(extent.width)
        .setHeight(extent.height)
        .setLayers(1);
    framebuffers_.emplace_back(vkc_->GetDevice().createFramebuffer(info));
  }
}

void SceneRenderPass::CreateCommandBuffers() {
  auto vkgl = vkg_.lock();
  if (!vkgl) {
    throw std::runtime_error("Failed to lock vkg when creating command buffers");
  }

  vk::CommandBufferAllocateInfo info;
  info.setCommandPool(vkgl->GetCommandPool())
      .setCommandBufferCount(vkgl->GetSwapchainSize())
      .setLevel(vk::CommandBufferLevel::ePrimary);
  command_buffers_ = vkc_->GetDevice().allocateCommandBuffers(info);
}

vk::DescriptorPool SceneRenderPass::GetDescriptorPool() {
  return uniform_descriptor_pool_;
}

void SceneRenderPass::CreateDescriptorPool() {
  vk::DescriptorPoolCreateInfo pci;
  vk::DescriptorPoolSize pool_size;
  // HACK: 128,32 is a trivial number here.
  pool_size.setDescriptorCount(128).setType(vk::DescriptorType::eUniformBuffer);
  pci.setPoolSizes(pool_size).setMaxSets(32);
  uniform_descriptor_pool_ = vkc_->GetDevice().createDescriptorPool(pci);
}

SceneRenderPass::~SceneRenderPass() {
  auto device = vkc_->GetDevice();
  DestroySwapchain();
  // for (auto buffer : uniform_buffers_) {
  //   vkc_->UnmapMemory(buffer);
  //   vkc_->FreeBuffer(buffer);
  // }
  // uniform_buffers_.clear();
  device.destroy(uniform_descriptor_pool_);
  device.destroy(render_pass_);
}

void SceneRenderPass::DestroySwapchain() {
  auto device = vkc_->GetDevice();
  // Depth Resources
  device.destroy(depth_image_view_);
  vkc_->FreeImage(depth_image_);
  // Frame Buffers
  for (auto fb : framebuffers_) {
    device.destroyFramebuffer(fb);
  }
  framebuffers_.clear();
}

void SceneRenderPass::RecreateSwapchain() {
  DestroySwapchain();
  CreateDepthResources();
  CreateFramebuffers();
}

vk::CommandBuffer SceneRenderPass::Draw() {
  auto vkgl = vkg_.lock();
  if (!vkgl) {
    throw std::runtime_error("Cannot create render pass when vkg cannot lock.");
  }
  auto extent = vkgl->GetSwapchainExtent();
  auto current_index = vkgl->GetFrameIndex();
  vk::CommandBuffer current_command_buffer = command_buffers_[current_index];
  std::array<vk::ClearValue, 2> clear_value
      = {background_color_, depth_stencil_value_};
  // Begin Command Buffer
  vk::CommandBufferBeginInfo begin_info;
  begin_info.setFlags(vk::CommandBufferUsageFlagBits::eSimultaneousUse);
  current_command_buffer.begin(begin_info);

  // Begin Render Pass
  vk::RenderPassBeginInfo render_pass_info;
  render_pass_info.renderPass = render_pass_;
  render_pass_info.framebuffer = framebuffers_[vkgl->GetAcquiredImageIndex()];
  render_pass_info.renderArea.extent = extent;
  render_pass_info.renderArea.offset.setX(0);
  render_pass_info.renderArea.offset.setY(0);
  render_pass_info.setClearValues(clear_value);
  current_command_buffer.beginRenderPass(render_pass_info,
                                         vk::SubpassContents::eInline);

  for (const auto& pipeline : pipelines_) {
    pipeline->Draw(current_command_buffer);
  }

  current_command_buffer.endRenderPass();
  current_command_buffer.end();
  return current_command_buffer;
}

void SceneRenderPass::AddPipeline(std::shared_ptr<ScenePipelineBase> pipeline) {
  pipelines_.push_back(std::move(pipeline));
}

vk::RenderPass SceneRenderPass::GetRenderPass() { return render_pass_; }

}  // namespace axes::gui
