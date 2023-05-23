#pragma once
#include <vulkan/vulkan.hpp>

namespace axes::gui {

class RenderPassBase {
public:
  virtual void RecreateSwapchain() = 0;

  virtual vk::CommandBuffer Draw() = 0;

  virtual vk::DescriptorPool GetDescriptorPool() = 0;

  virtual ~RenderPassBase() = default;
};

}  // namespace axes::gui
