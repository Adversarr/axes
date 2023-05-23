#include "axes/gui/details/guisys.hpp"

#include "axes/core/ecs/ecs.hpp"
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/mesh_pipeline.hpp"
#include "axes/gui/tags.hpp"

namespace axes::gui {

GuiSystem::GuiSystem(std::shared_ptr<VkContext> vkc,
                     std::shared_ptr<VkGraphicsContext> vkg)
    : vkc_(vkc), vkg_(vkg), sbuffer_(vkc) {
  // TODO: Other initialize codes goes here.
}

// TODO: Impl
GuiSystem::~GuiSystem() = default;

void GuiSystem::TickLogic() {
  // TODO: Foreach Simplical Render Config.

  // Foreach Simplical Render Data
  ecs::ComponentManager<SimplicalRenderData> cm;
  for (const auto &[k, v] : cm) {
    if (!v->flush_) {
      continue;
    }

    // TODO: Replace with Data Provider.
    // Get the component. and fill in all the required buffers.

    // 1. Fill the vertex buffer.
    CreateSceneSharedData(k, v);
    // 2. Fill each pipeline's data
    CreateSceneMeshData(k, v);
  }
}

void GuiSystem::TickRender() {
  throw std::runtime_error("No Implementation");
}

void GuiSystem::CreateSceneMeshData(ecs::EntityID ent, SimplicalRenderData *data) {
  // TODO: Implementation
}

void GuiSystem::CreateSceneSharedData(ecs::EntityID ent,
                                      SimplicalRenderData *data) {
  ecs::ComponentManager<details::SceneRenderDataSharedGpu> sr;

  // 1. For shared part, i.e. vertex buffer, fill in directly.
  auto *shared = sr.EmplaceComponent(ent, vkc_);
  size_t vertex_count = data->vertices_.Size();
  if (vertex_count == 0) {
    throw std::runtime_error("Error: No Vertex Provided for entity "
                             + std::to_string(ent));
  }
  // FIXME: 
  // vkc_->PrepareBuffer(shared->vertex_buffer_,
  //                     vertex_count * sizeof(details::SceneVertex));
  sbuffer_.CopyBuffer(shared->vertex_buffer_, data->vertices_.Ptr(),
                      sizeof(details::SceneVertex) * data->vertices_.Size());
}

}  // namespace axes::gui
