#pragma once

#include "acore/ecs/ecs.hpp"
#include "agui/details/common.hpp"
#include "agui/details/vkcontext.hpp"
#include "agui/tags.hpp"

namespace axes::gui {

class GuiSystem : public ecs::SystemBase {
public:
  // NOTE: This system should have the lowest priority.

  GuiSystem(std::shared_ptr<VkContext> vkc,
            std::shared_ptr<VkGraphicsContext> vkg);

  ~GuiSystem();

  /**
   * @brief Prepare the render data, fill in all the required informations.
   *
   */
  void TickLogic() final;

  /**
   * @brief Submit the final render command.
   *
   */
  void TickRender() final;

  /**
   * @brief Reset all configurations to default, including camera, lighting, ...
   *
   */
  void Reset() final;

  /**
   * @brief Initialize just make the system reset to default.
   *
   */
  void Initialize() final;


private:
  void PrepareBuffer(VmaAllocBuffer &previous, size_t required_size);

  void CopyBuffer(VmaAllocBuffer dst_buffer, void* data, size_t nbytes);

  void CreateSBuffer();

  void CreateSceneSharedData(ecs::EntityID ent, SimplicalRenderData *data);

  // enum class SBufferStatus {
  //   kTooLarge,
  //   kFull,
  //   kOk
  // };

  struct SBufferTransferInfo {
    vk::Buffer dst_;
    vk::DeviceSize size_;
    vk::DeviceSize sb_offset_;
    vk::DeviceSize dst_offset_;
  };

  void SBufferAppend(void* data, size_t nbytes);
  void SBufferFlush();

  // XXX: maybe more functions here...
  std::shared_ptr<VkContext> vkc_;
  std::shared_ptr<VkGraphicsContext> vkg_;
  
  // staging buffer
  VmaAllocBuffer staging_buffer_;
  size_t staging_buffer_usage_;
  std::vector<SBufferTransferInfo> sb_tfinfo_;
};

} // namespace axes::gui