#pragma once
#include "axes/gui/details/scene_pipeline_base.hpp"
#include "axes/gui/details/vkcontext.hpp"

namespace axes::gui::details {

class PointPipeline : public ScenePipelineBase {
public:
  void Draw(vk::CommandBuffer& buffer) final;

  void CreateEntityRenderData(ecs::EntityID ent, StagingBuffer& sbuffer) final;

  virtual ~PointPipeline();

  explicit PointPipeline(std::shared_ptr<VkContext> vkc,
                         std::weak_ptr<VkGraphicsContext> vkg,
                         std::weak_ptr<SceneRenderPass> render_pass);

protected:
  void CreatePipeline(vk::RenderPass pass) final;
};

}  // namespace axes::gui::details
