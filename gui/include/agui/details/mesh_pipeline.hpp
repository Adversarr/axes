#pragma once
#include "agui/details/scene_pipeline_base.hpp"
#include "agui/details/vkcontext.hpp"

namespace axes::gui::details {

class MeshPipeline : public ScenePipelineBase {
public:
  virtual void Draw(vk::CommandBuffer &buffer) final;

  virtual ~MeshPipeline();

  explicit MeshPipeline(std::shared_ptr<VkContext> vkc,
                        std::weak_ptr<VkGraphicsContext> vkg,
                        SceneRenderPass *render_pass);

protected:
  void CreatePipeline(vk::RenderPass pass) final;
};

} // namespace axes::gui::details