#pragma once

#include "axes/core/ecs/ecs.hpp"
#include "axes/gui/details/common.hpp"
#include "axes/gui/details/staging_buffer.hpp"
#include "axes/gui/details/vkcontext.hpp"
#include "axes/gui/tags.hpp"

namespace axes::gui {

class GuiSystem : public ecs::SystemBase {
public:
  // NOTE: This system should have the lowest priority.

  GuiSystem(std::shared_ptr<VkContext> vkc, std::shared_ptr<VkGraphicsContext> vkg);

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
  // TODO: void Reset() final;

  /**
   * @brief Initialize just make the system reset to default.
   *
   */
  // TODO: void Initialize() final;

private:
  void CreateSceneSharedData(ecs::EntityID ent, SimplicalRenderData* data);

  void CreateSceneMeshData(ecs::EntityID ent, SimplicalRenderData* data);

  // XXX: maybe more functions here...
  std::shared_ptr<VkContext> vkc_;
  std::shared_ptr<VkGraphicsContext> vkg_;
  std::vector<std::shared_ptr<ScenePipelineBase>> scene_pipelines_;

  // staging buffer
  StagingBuffer sbuffer_;
};

}  // namespace axes::gui
