// // Uncomment if you need to debug VMA.
// #define VMA_DEBUG_LOG(format, ...) do { \
//        printf(format __VA_OPT__(,) __VA_ARGS__); \
//        printf("\n"); \
//    } while(false)

#define VMA_IMPLEMENTATION
#include "axes/gui/details/vkcontext.hpp"

#include <fmt/format.h>

#include <axes/core/utils/common.hpp>
#include <axes/core/utils/log.hpp>
#include <set>

VKAPI_ATTR VkResult VKAPI_CALL vkCreateDebugUtilsMessengerEXT(
    VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT *pCreateInfo,
    const VkAllocationCallbacks *pAllocator, VkDebugUtilsMessengerEXT *pMessenger) {
  auto func = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
      vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));
  if (func != nullptr) {
    return func(instance, pCreateInfo, pAllocator, pMessenger);
  } else {
    return VK_ERROR_EXTENSION_NOT_PRESENT;
  }
}

VKAPI_ATTR void VKAPI_CALL vkDestroyDebugUtilsMessengerEXT(
    VkInstance instance, VkDebugUtilsMessengerEXT messenger,
    VkAllocationCallbacks const *pAllocator) {
  auto func = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
      vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT"));
  if (func != nullptr) {
    func(instance, messenger, pAllocator);
  }
}

VKAPI_ATTR VkBool32 VKAPI_CALL
debug_message_callback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
                       VkDebugUtilsMessageTypeFlagsEXT messageTypes,
                       VkDebugUtilsMessengerCallbackDataEXT const *pCallbackData,
                       void * /*pUserData*/) {
  auto cstr_to_string
      = [](const char *c) { return (c == nullptr) ? "Null" : std::string_view(c); };

  fmt::memory_buffer buffer;
  fmt::format_to(
      std::back_inserter(buffer),
      "{}: {}:\n"
      "messageIDName   = <{}>\n"
      "messageIdNumber = {}\n"
      "message         = <{}>\n",
      vk::to_string(
          static_cast<vk::DebugUtilsMessageSeverityFlagBitsEXT>(messageSeverity)),
      vk::to_string(static_cast<vk::DebugUtilsMessageTypeFlagsEXT>(messageTypes)),
      cstr_to_string(pCallbackData->pMessageIdName), pCallbackData->messageIdNumber,
      cstr_to_string(pCallbackData->pMessage));

  if (0 < pCallbackData->queueLabelCount) {
    fmt::format_to(std::back_inserter(buffer), "\tQueue Labels:\n");
    for (uint32_t i = 0; i < pCallbackData->queueLabelCount; i++) {
      fmt::format_to(std::back_inserter(buffer), "\t\tlabelName = <{}>\n",
                     cstr_to_string(pCallbackData->pQueueLabels[i].pLabelName));
    }
  }
  if (0 < pCallbackData->cmdBufLabelCount) {
    fmt::format_to(std::back_inserter(buffer), "\tCommandBuffer Labels:\n");
    for (uint32_t i = 0; i < pCallbackData->cmdBufLabelCount; i++) {
      fmt::format_to(std::back_inserter(buffer), "\t\tlabelName = <{}>\n",
                     cstr_to_string(pCallbackData->pCmdBufLabels[i].pLabelName));
    }
  }
  if (0 < pCallbackData->objectCount) {
    fmt::format_to(std::back_inserter(buffer), "\tObjects:\n");
    for (uint32_t i = 0; i < pCallbackData->objectCount; i++) {
      fmt::format_to(std::back_inserter(buffer),
                     "\t\tObject {}\n"
                     "\t\t\tobjectType   = {}\n"
                     "\t\t\tobjectHandle = {}\n"
                     "\t\t\tobjectName   = <{}>\n",
                     i,
                     vk::to_string(static_cast<vk::ObjectType>(
                         pCallbackData->pObjects[i].objectType)),
                     pCallbackData->pObjects[i].objectHandle,
                     cstr_to_string(pCallbackData->pObjects[i].pObjectName));
    }
  }
  auto s = fmt::to_string(buffer);

  switch (messageSeverity) {
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT:
      AXES_ERROR("{}", s);
      break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT:
      AXES_WARN("{}", s);
      break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT:
      AXES_INFO("{}", s);
      break;
    case VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT:
      AXES_DEBUG("{}", s);
      break;
    default:
      // Not valid bit.
      break;
  };

  return false;
}

double rank_func(vk::PhysicalDevice device) {
  auto type = device.getProperties().deviceType;
  switch (type) {
    case vk::PhysicalDeviceType::eCpu:
      return 1.0;
    case vk::PhysicalDeviceType::eIntegratedGpu:
      return 3.0;
    case vk::PhysicalDeviceType::eVirtualGpu:
      return 4.0;
    case vk::PhysicalDeviceType::eDiscreteGpu:
      return 5.0;
    default:
      return 0.0;
  }
}

namespace axes::gui {

VkContext::VkContext(VkContextCreateInfo info) {
  create_info_ = info;
  CreateInstance();
  CreatePhysicalDevice();
  CreateDevice();

  // Create vma allocator with auto usage.
  VmaAllocatorCreateInfo vaci;
  memset(&vaci, 0, sizeof(vaci));  // Force set to zero for correct init.
  vaci.device = device_;
  vaci.physicalDevice = physical_device_;
  vaci.instance = instance_;
  vaci.vulkanApiVersion = create_info_.api_version_;

  if (vmaCreateAllocator(&vaci, &default_allocator_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create Vma Allocator.");
  }

  // create transient_command_pool :
  vk::CommandPoolCreateInfo cpci;
  cpci.setFlags(vk::CommandPoolCreateFlagBits::eTransient
                | vk::CommandPoolCreateFlagBits::eResetCommandBuffer);
  transient_command_pool_ = device_.createCommandPool(cpci);
}

VkContext::~VkContext() {
  device_.destroy(transient_command_pool_);
  vmaDestroyAllocator(default_allocator_);
  device_.destroy();
  if (debug_messenger_) {
    instance_.destroy(debug_messenger_);
  }
  if (surface_) {
    instance_.destroy(surface_);
  }
  instance_.destroy();
}

void VkContext::CreateInstance() {
  system_info_.available_layers_ = vk::enumerateInstanceLayerProperties();
  system_info_.available_extensions_ = vk::enumerateInstanceExtensionProperties();
  system_info_.validation_layers_available_
      = CheckLayerSupport({"VK_LAYER_KHRONOS_validation"});
  system_info_.debug_utils_available_
      = CheckExtensionSupport({VK_EXT_DEBUG_UTILS_EXTENSION_NAME});
  if (create_info_.enable_validation_
      && !system_info_.validation_layers_available_) {
    for (const auto &lay : system_info_.available_layers_) {
      std::cout << lay.layerName << std::endl;
    }
    throw std::runtime_error(
        "Vulkan validation required with validation layer unavailable");
  }

  if (create_info_.enable_validation_ && !system_info_.debug_utils_available_) {
    throw std::runtime_error(
        "Vulkan validation required with debug utils unavailable");
  }

  vk::ApplicationInfo app_info;
  app_info.setPApplicationName(create_info_.app_name_.c_str());
  app_info.setApplicationVersion(create_info_.app_version_);
  app_info.setEngineVersion(create_info_.engine_version_);
  app_info.setApiVersion(create_info_.api_version_);
  vk::InstanceCreateInfo create_info;
  create_info.setPApplicationInfo(&app_info);

  // Setup Extensions.
  std::vector<const char *> extensions;
  if (create_info_.glfw_window_) {
    AXES_INFO("Vulkan Context Initialize with GLFW Window.");
    uint32_t glfw_extension_count = 0;
    const char **glfw_extensions;
    glfw_extensions = glfwGetRequiredInstanceExtensions(&glfw_extension_count);
    std::copy(glfw_extensions, glfw_extensions + glfw_extension_count,
              std::back_inserter(extensions));
  } else {
    AXES_INFO("Vulkan Contaxt Initialize without GLFW.");
  }
  if (create_info_.enable_validation_) {
    extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  }
  if constexpr (utils::get_platform_type() == utils::PlatformType::kApple) {
    extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
  }
  AXES_CHECK(CheckExtensionSupport(extensions), "Some extensions not support");
  create_info.setPEnabledExtensionNames(extensions);

  // If apple, set flag.
  if constexpr (utils::get_platform_type() == utils::PlatformType::kApple) {
    create_info.setFlags(vk::InstanceCreateFlagBits::eEnumeratePortabilityKHR);
  }

  auto &enabled_layers = create_info_.enabled_layers_;
  if (create_info_.enable_validation_) {
    enabled_layers.push_back("VK_LAYER_KHRONOS_validation");
  }
  AXES_CHECK(CheckLayerSupport(enabled_layers), "Some layer not support");
  create_info.setPEnabledLayerNames(enabled_layers);

  std::ostringstream oss;
  oss << "Creating vulkan instance with layers: " << std::endl;
  for (const auto *lay : enabled_layers) {
    oss << "\t" << lay << std::endl;
  }
  AXES_INFO("{}", oss.str());
  instance_ = vk::createInstance(create_info);

  if (create_info_.enable_validation_) {
    vk::DebugUtilsMessengerCreateInfoEXT info;
    info.messageSeverity = vk::DebugUtilsMessageSeverityFlagBitsEXT::eError
                           | vk::DebugUtilsMessageSeverityFlagBitsEXT::eWarning;
    info.messageType = vk::DebugUtilsMessageTypeFlagBitsEXT::eGeneral
                       | vk::DebugUtilsMessageTypeFlagBitsEXT::ePerformance
                       | vk::DebugUtilsMessageTypeFlagBitsEXT::eValidation;
    info.pfnUserCallback = debug_message_callback;
    info.pUserData = nullptr;  // Optional
    debug_messenger_ = instance_.createDebugUtilsMessengerEXT(info);
  }
}

bool VkContext::CheckLayerSupport(const std::vector<const char *> &layers_require) {
  for (const auto &layer_name : layers_require) {
    bool found = false;
    for (const auto &layer_prop : system_info_.available_layers_) {
      if (strcmp(layer_name, layer_prop.layerName) != 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

bool VkContext::CheckExtensionSupport(
    const std::vector<const char *> &extensions_require) {
  for (const auto &ext_name : extensions_require) {
    bool found = false;
    for (const auto &ext_name_sup : system_info_.available_extensions_) {
      if (strcmp(ext_name, ext_name_sup.extensionName) != 0) {
        found = true;
        break;
      }
    }
    if (!found) {
      return false;
    }
  }
  return true;
}

void VkContext::CreatePhysicalDevice() {
  auto &e = create_info_.enabled_device_extensions_;
  if constexpr (axes::utils::get_platform_type() == utils::PlatformType::kApple) {
    if (auto it = std::find_if(e.begin(), e.end(),
                               [](const char *ext) {
                                 return strcmp(ext, "VK_KHR_portability_subset")
                                        == 0;
                               });
        it == e.end()) {
      e.push_back("VK_KHR_portability_subset");
    }
  }
  create_info_.headless_mode_ |= (create_info_.glfw_window_ == nullptr);
  if (!create_info_.headless_mode_) {
    // Initialize surface.
    auto *win = create_info_.glfw_window_;
    surface_ = win->CreateSurface(instance_);
    if (auto it = std::find_if(e.begin(), e.end(),
                               [](const char *ext) {
                                 return strcmp(ext, VK_KHR_SWAPCHAIN_EXTENSION_NAME)
                                        == 0;
                               });
        it == e.end()) {
      e.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    }
    create_info_.require_present_queue_ = true;
  } else {
    create_info_.require_present_queue_ = false;
  }
  auto devices = instance_.enumeratePhysicalDevices();

  std::vector<vk::PhysicalDevice> available_devices;
  for (const auto &d : devices) {
    SystemInfo::PhysicalDeviceInfo device_info = GetDeviceInfo(d);
    std::set<std::string_view> required(e.begin(), e.end());
    AXES_DEBUG("Found Physical Device: {}", device_info.properties_.deviceName);
    for (const auto &p : device_info.extension_properties_) {
      required.erase(p.extensionName);
    }
    if (!required.empty()) {
      continue;
    }

    if (!device_info.graphics_family_.has_value()) {
      continue;
    }
    if (create_info_.require_present_queue_
        && !device_info.present_family_.has_value()) {
      continue;
    }
    if (create_info_.require_compute_queue_
        && !device_info.compute_family_.has_value()) {
      continue;
    }
    if (create_info_.require_transfer_queue_
        && !device_info.transfer_family_.has_value()) {
      continue;
    }

    if (!create_info_.headless_mode_) {
      // Check Swapchain Support
      if (device_info.surface_formats_.empty()) {
        continue;
      }

      if (device_info.surface_present_modes_.empty()) {
        continue;
      }
    }
    available_devices.push_back(d);
  }

  if (!create_info_.rank_function_.has_value()) {
    create_info_.rank_function_ = rank_func;
  }

  std::sort(available_devices.begin(), available_devices.end(),
            [rank = create_info_.rank_function_.value()](
                const vk::PhysicalDevice &l, const vk::PhysicalDevice &r) {
              return rank(l) > rank(r);
            });
  AXES_CHECK(!available_devices.empty(), "No device available.");
  physical_device_ = available_devices.front();
  system_info_.physical_device_info_ = GetDeviceInfo(physical_device_);

  AXES_INFO("Physical device picked. {}",
            system_info_.physical_device_info_.properties_.deviceName);
}

SystemInfo::PhysicalDeviceInfo VkContext::GetDeviceInfo(
    vk::PhysicalDevice device) const {
  SystemInfo::PhysicalDeviceInfo info;
  info.properties_ = device.getProperties();
  info.extension_properties_ = device.enumerateDeviceExtensionProperties();
  int i = 0;
  for (const auto &family : device.getQueueFamilyProperties()) {
    if (family.queueCount > 0) {
      if (!(info.graphics_family_)
          && family.queueFlags & vk::QueueFlagBits::eGraphics) {
        info.graphics_family_ = i;
      }
      if (!(info.compute_family_)
          && family.queueFlags & vk::QueueFlagBits::eCompute) {
        info.compute_family_ = i;
      }
      if (!(info.transfer_family_)
          && family.queueFlags & vk::QueueFlagBits::eTransfer) {
        info.transfer_family_ = i;
      }
    }

    if (surface_) {
      bool has_surface_support = device.getSurfaceSupportKHR(i, surface_);
      if (has_surface_support) {
        if (!info.present_family_) {
          info.present_family_ = i;
        }
      }

      info.surface_formats_ = device.getSurfaceFormatsKHR(surface_);
      info.surface_present_modes_ = device.getSurfacePresentModesKHR(surface_);
    }
    i += 1;
  }

  return info;
}

void VkContext::CreateDevice() {
  std::vector<vk::DeviceQueueCreateInfo> queue_create_info;
  std::set<uint32_t> unique_queue_families;
  // Foreach Family, if required, add to set:
  unique_queue_families.emplace(
      system_info_.physical_device_info_.graphics_family_.value());
  if (create_info_.require_present_queue_) {
    unique_queue_families.emplace(
        system_info_.physical_device_info_.present_family_.value());
  }
  if (create_info_.require_compute_queue_) {
    unique_queue_families.emplace(
        system_info_.physical_device_info_.compute_family_.value());
  }
  if (create_info_.require_transfer_queue_) {
    unique_queue_families.emplace(
        system_info_.physical_device_info_.transfer_family_.value());
  }

  float queue_priority = 1.0f;
  for (auto family : unique_queue_families) {
    vk::DeviceQueueCreateInfo info;
    info.setQueueFamilyIndex(family).setQueueCount(1).setPQueuePriorities(
        &queue_priority);
    queue_create_info.emplace_back(info);
  }

  vk::DeviceCreateInfo device_create_info;
  device_create_info.setQueueCreateInfos(queue_create_info)
      .setPEnabledFeatures(&create_info_.physical_device_features_)
      .setPEnabledExtensionNames(create_info_.enabled_device_extensions_)
      .setPEnabledLayerNames(create_info_.enabled_layers_);

  device_ = physical_device_.createDevice(device_create_info);
  // setup queue:
  if (system_info_.physical_device_info_.graphics_family_) {
    graphics_queue_ = device_.getQueue(
        system_info_.physical_device_info_.graphics_family_.value(), 0);
  }
  if (system_info_.physical_device_info_.present_family_
      && create_info_.require_present_queue_) {
    present_queue_ = device_.getQueue(
        system_info_.physical_device_info_.present_family_.value(), 0);
  }
  if (system_info_.physical_device_info_.compute_family_
      && create_info_.require_compute_queue_) {
    compute_queue_ = device_.getQueue(
        system_info_.physical_device_info_.compute_family_.value(), 0);
  }
  if (system_info_.physical_device_info_.transfer_family_
      && create_info_.require_transfer_queue_) {
    transfer_queue_ = device_.getQueue(
        system_info_.physical_device_info_.transfer_family_.value(), 0);
  }
}

void VkContext::RunTransientCommandInplace(
    const std::function<void(vk::CommandBuffer)> &f, vk::Queue q) {
  vk::CommandBufferAllocateInfo alloc_info{};
  alloc_info.level = vk::CommandBufferLevel::ePrimary;
  alloc_info.commandPool = transient_command_pool_;
  alloc_info.commandBufferCount = 1;
  auto buf = device_.allocateCommandBuffers(alloc_info).front();
  vk::CommandBufferBeginInfo begin_info{};
  begin_info.flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit;
  buf.begin(begin_info);
  f(buf);
  buf.end();
  vk::SubmitInfo submit_info;
  submit_info.setCommandBuffers(buf);
  q.submit(submit_info);
  q.waitIdle();
  device_.freeCommandBuffers(transient_command_pool_, buf);
}

VmaAllocBuffer VkContext::AllocateBuffer(vk::BufferCreateInfo buf_info,
                                         VmaAllocationCreateInfo info) {
  AXES_TRACE("Allocate buffer: usage = {}, size = {}",
             vk::to_string(buf_info.usage), buf_info.size);
  VkBufferCreateInfo bfci = buf_info;
  VmaAllocBuffer buffer;
  vk::Result result{vmaCreateBuffer(default_allocator_, &bfci, &info,
                                    &buffer.buffer_, &buffer.alloc_,
                                    &buffer.alloc_info_)};
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("Failed to allocate image. error="
                             + vk::to_string(result));
  }
  return buffer;
}

void VkContext::FreeBuffer(VmaAllocBuffer buf) {
  device_.waitIdle();
  vmaDestroyBuffer(default_allocator_, buf.buffer_, buf.alloc_);
}

VmaAllocImage VkContext::AllocateImage(vk::ImageCreateInfo img_info,
                                       VmaAllocationCreateInfo info) {
  VkImageCreateInfo ici = img_info;
  VmaAllocImage img;
  vk::Result result{vmaCreateImage(default_allocator_, &ici, &info, &img.image_,
                                   &img.alloc_, &img.alloc_info_)};
  if (result != vk::Result::eSuccess) {
    throw std::runtime_error("Failed to allocate image. error="
                             + vk::to_string(result));
  }
  return img;
}

void VkContext::FreeImage(VmaAllocImage img) {
  vmaDestroyImage(default_allocator_, img.image_, img.alloc_);
}

void *VkContext::MapMemory(const VmaAllocBuffer &buffer) {
  void *ptr = nullptr;
  auto vkr = vk::Result{vmaMapMemory(default_allocator_, buffer.alloc_, &ptr)};
  if (vkr != vk::Result::eSuccess) {
    throw std::runtime_error("Failed to map memory: " + vk::to_string(vkr));
  }
  return ptr;
}

void VkContext::UnmapMemory(const VmaAllocBuffer &buffer) {
  vmaUnmapMemory(default_allocator_, buffer.alloc_);
}

void VkContext::PrepareBuffer(VmaAllocBuffer &buffer, vk::BufferCreateInfo bci,
                              VmaAllocationCreateInfo vaci) {
  if (buffer.buffer_ != nullptr) {
    auto size = buffer.alloc_info_.size;
    if (size >= bci.size) {
      // The buffer is large enough for current requirement.
      return;
    }
    // Otherwise, free the original buffer
    FreeBuffer(buffer);
  }

  // Create buffer fulfilling the requirement.
  buffer = AllocateBuffer(bci, vaci);
}

}  // namespace axes::gui
