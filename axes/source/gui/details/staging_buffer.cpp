#include "axes/gui/details/staging_buffer.hpp"
namespace axes::gui {

StagingBuffer::StagingBuffer(std::shared_ptr<VkContext> vkc) : vkc_(vkc) {
  // TODO: Allocate the staging buffer.
  vk::BufferCreateInfo bci;
  bci.setUsage(vk::BufferUsageFlagBits::eTransferSrc);
  // 64MB = (64 << 20)
  bci.setSize(64 << 20);
  bci.setSharingMode(vk::SharingMode::eExclusive);

  VmaAllocationCreateInfo alci{};
  memset(&alci, 0, sizeof(alci));
  alci.usage = VMA_MEMORY_USAGE_AUTO;
  alci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
               | VMA_ALLOCATION_CREATE_MAPPED_BIT;
  buffer_ = vkc_->AllocateBuffer(bci, alci);
}

StagingBuffer::~StagingBuffer() { vkc_->FreeBuffer(buffer_); }

void StagingBuffer::Flush() {
  vkc_->RunTransientCommandInplace(
      [this](vk::CommandBuffer cbuffer) {
        for (auto st : sb_tfinfo_) {
          vk::BufferCopy bc;
          bc.setSize(st.size_)
              .setDstOffset(st.dst_offset_)
              .setSrcOffset(st.sb_offset_);
          cbuffer.copyBuffer(buffer_.buffer_, st.dst_, bc);
        }
      },
      vkc_->GetGraphicsQueue());
  usage_ = 0;
  sb_tfinfo_.clear();
}

void StagingBuffer::CopyBuffer(VmaAllocBuffer dst_buffer, void* data,
                               size_t nbytes) {
  size_t sbsize = buffer_.alloc_info_.size;
  if (dst_buffer.buffer_ == VK_NULL_HANDLE) {
    throw std::runtime_error("buffer is null handle.");
  }
  if (nbytes + usage_ > buffer_.alloc_info_.size) {
    Flush();
  }

  if (nbytes > buffer_.alloc_info_.size) [[unlikely]] {
    size_t copy_times = (nbytes + sbsize - 1) / sbsize;
    char* large_data = static_cast<char*>(data);
    for (size_t i = 0; i < copy_times; ++i) {
      // For the last turn, avoid memory error.
      size_t nb_copy = std::min(sbsize, nbytes - i * copy_times);
      Append(large_data + i * sbsize, nb_copy);
      SBufferTransferInfo ti;
      ti.size_ = nb_copy;
      ti.dst_ = dst_buffer.buffer_;
      ti.sb_offset_ = 0;
      ti.dst_offset_ = i * sbsize;
      sb_tfinfo_.push_back(ti);
      Flush();
    }
  } else {
    SBufferTransferInfo ti;
    ti.size_ = nbytes;
    ti.dst_ = dst_buffer.buffer_;
    ti.sb_offset_ = usage_;
    ti.dst_offset_ = 0;
    sb_tfinfo_.push_back(ti);
    Append(data, nbytes);
  }
}

void StagingBuffer::Append(void* data, size_t nbytes) {
  char* mapped_data = static_cast<char*>(buffer_.alloc_info_.pMappedData);
  memcpy(mapped_data + usage_, data, nbytes);
  usage_ += nbytes;
}

}  // namespace axes::gui
