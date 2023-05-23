#pragma once
#include <memory>

#include "axes/gui/details/vkcontext.hpp"
namespace axes::gui {

class StagingBuffer {
public:
  explicit StagingBuffer(std::shared_ptr<VkContext> vkc);

  ~StagingBuffer();

  void CopyBuffer(VmaAllocBuffer buffer, void* data, size_t nbytes);

  void Flush();

  void Append(void* data, size_t nbytes);

  struct SBufferTransferInfo {
    vk::Buffer dst_;
    vk::DeviceSize size_;
    vk::DeviceSize sb_offset_;
    vk::DeviceSize dst_offset_;
  };

private:
  std::shared_ptr<VkContext> vkc_;
  VmaAllocBuffer buffer_;
  size_t usage_;
  std::vector<SBufferTransferInfo> sb_tfinfo_;
};

}  // namespace axes::gui
