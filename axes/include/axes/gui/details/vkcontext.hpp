#pragma once
#include <vma/vk_mem_alloc.h>
#include <optional>

#include "axes/gui/details/glfwin.hpp"

namespace axes::gui {

struct VmaAllocBuffer {
  VmaAllocation alloc_;
  VmaAllocationInfo alloc_info_;
  VkBuffer buffer_{nullptr};
};

struct VmaAllocImage {
  VmaAllocation alloc_;
  VmaAllocationInfo alloc_info_;
  VkImage image_;
};

struct VkContextCreateInfo {
  std::string app_name_;
  std::string engine_name_;
  uint32_t api_version_{VK_API_VERSION_1_3};
  uint32_t app_version_{VK_MAKE_VERSION(1, 0, 0)};
  uint32_t engine_version_{VK_MAKE_VERSION(1, 0, 0)};
  bool enable_validation_{true};
  bool headless_mode_{false};
  bool require_graphics_queue_ = true;
  bool require_present_queue_ = true;
  bool require_compute_queue_ = false;
  bool require_transfer_queue_ = false;
  std::vector<const char *> enabled_layers_;
  std::vector<const char *> enabled_device_extensions_;
  vk::PhysicalDeviceFeatures physical_device_features_;
  std::optional<std::function<double(vk::PhysicalDevice)>> rank_function_;
  GlfwWindow *glfw_window_{nullptr};
};

struct SystemInfo {
  std::vector<vk::LayerProperties> available_layers_;
  std::vector<vk::ExtensionProperties> available_extensions_;
  bool validation_layers_available_ = false;
  bool debug_utils_available_ = false;

  struct PhysicalDeviceInfo {
    vk::PhysicalDeviceProperties properties_;
    std::vector<vk::ExtensionProperties> extension_properties_;
    std::optional<uint32_t> graphics_family_;
    std::optional<uint32_t> present_family_;
    std::optional<uint32_t> compute_family_;
    std::optional<uint32_t> transfer_family_;

    std::vector<vk::SurfaceFormatKHR> surface_formats_;
    std::vector<vk::PresentModeKHR> surface_present_modes_;
  } physical_device_info_;
};

class VkContext {
public:
  explicit VkContext(VkContextCreateInfo info);
  VkContext(const VkContext &) = delete;
  VkContext(VkContext &&) = delete;
  ~VkContext();

  /**
   * @brief Get the vk::Instance object
   *
   * @return
   */
  vk::Instance GetInstance() { return instance_; }

  /**
   * @brief Get the vk::Device object.
   *
   * @return
   */
  vk::Device GetDevice() { return device_; }

  /**
   * @brief Get the vk::PhysicalDevice object.
   *
   * @return
   */
  vk::PhysicalDevice GetPhysicalDevice() { return physical_device_; }

  /**
   * @brief Get the basic system information
   *
   * @return
   */
  const SystemInfo &GetSystemInfo() const noexcept { return system_info_; }

  /**
   * @brief Get the transient command pool.
   *
   * @return
   */
  vk::CommandPool GetTransientCommandPool() const noexcept {
    return transient_command_pool_;
  }

  /**
   * @brief Get the Present Surface
   *
   * @return
   */
  vk::SurfaceKHR GetSurface() { return surface_; }

  /**
   * @brief Return the graphics queue.
   *
   * @return
   */
  vk::Queue GetGraphicsQueue() { return graphics_queue_; }

  /**
   * @brief Return the present queue.
   *
   * @return
   */
  vk::Queue GetPresentQueue() { return present_queue_; }

  /**
   * @brief Create and run a transient command.
   *
   * @param f function records commands on the CommandBuffer.
   * @param q which queue to run the command
   */
  void RunTransientCommandInplace(const std::function<void(vk::CommandBuffer)> &f,
                                  vk::Queue q);

  void PrepareBuffer(VmaAllocBuffer &buffer, vk::BufferCreateInfo bci,
                     VmaAllocationCreateInfo vaci);

  [[nodiscard]] VmaAllocBuffer AllocateBuffer(vk::BufferCreateInfo buf_info,
                                              VmaAllocationCreateInfo info);

  void *MapMemory(const VmaAllocBuffer &buffer);

  void UnmapMemory(const VmaAllocBuffer &buffer);

  void FreeBuffer(VmaAllocBuffer buffer);

  [[nodiscard]] VmaAllocImage AllocateImage(vk::ImageCreateInfo img_info,
                                            VmaAllocationCreateInfo info);

  void FreeImage(VmaAllocImage img);

private:
  bool CheckLayerSupport(const std::vector<const char *> &layers_require);

  bool CheckExtensionSupport(const std::vector<const char *> &extensions_require);

  /**
   * @brief Create vulkan instance.
   */
  void CreateInstance();

  /**
   * @brief physical device
   */
  void CreatePhysicalDevice();

  /**
   * @brief logical device.
   */
  void CreateDevice();

  SystemInfo::PhysicalDeviceInfo GetDeviceInfo(vk::PhysicalDevice device) const;

  VmaAllocator default_allocator_;
  VkContextCreateInfo create_info_;
  SystemInfo system_info_;
  vk::Instance instance_;
  vk::DebugUtilsMessengerEXT debug_messenger_;
  vk::PhysicalDevice physical_device_;
  vk::Device device_;
  // Queue created.
  vk::Queue graphics_queue_;
  vk::Queue present_queue_;
  vk::Queue compute_queue_;
  vk::Queue transfer_queue_;

  vk::CommandPool transient_command_pool_;

  vk::SurfaceKHR surface_;
};
}  // namespace axes::gui
