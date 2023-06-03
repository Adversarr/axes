#include "axes/gui/details/guisys.hpp"

#include <imgui.h>

#include "axes/core/ecs/resource_manager.hpp"
#include "axes/core/utils/log.hpp"
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/mesh_pipeline.hpp"
#include "axes/gui/details/scene_data.hpp"
#include "axes/gui/details/scene_pass.hpp"
#include "axes/gui/details/ui_pass.hpp"
#include "axes/gui/details/vkgraphics.hpp"
#include "axes/gui/tags.hpp"

namespace axes::gui {

void GuisysKeymap::InitResource() {
  // TODO: 
}

void default_gui_render() {
  // TODO: more implement
  ImGui::Begin("UI");
  bool update = false;
  auto cam = ecs::Rc<SceneCamera>{}.MakeValid();
  std::string cp = fmt::format("Camera Position {} {} {}", cam->position_.x(),
                               cam->position_.y(), cam->position_.z());
  if (ImGui::TreeNode(cp.c_str())) {
    update |= ImGui::InputDouble("X", &cam->position_.x());
    update |= ImGui::InputDouble("Y", &cam->position_.y());
    update |= ImGui::InputDouble("Z", &cam->position_.z());
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Camera Front")) {
    update |= ImGui::InputDouble("X", &cam->front_.x());
    update |= ImGui::InputDouble("Y", &cam->front_.y());
    update |= ImGui::InputDouble("Z", &cam->front_.z());
    ImGui::TreePop();
  }
  if (update) {
    cam->update_ = true;
  }
  auto light = ecs::Rc<SceneLight>{}.MakeValid();
  auto proj = ecs::Rc<SceneProjection>{}.MakeValid();
  ImGui::End();
}

static void setup_default_keymap() {
  auto km = ecs::Rc<GuisysKeymap>{}.MakeValid();
  // TODO: register hijk, wasd
}

GuiSystem::GuiSystem(std::shared_ptr<GlfwWindow> win,
                     std::shared_ptr<VkContext> vkc,
                     std::shared_ptr<VkGraphicsContext> vkg)
    : ecs::SystemBase(true, ecs::SystemBasePriority::kLowest),
      vkc_(vkc),
      vkg_(vkg),
      win_(win),
      sbuffer_(vkc) {
  scene_rp_ = std::make_shared<SceneRenderPass>(vkc_, vkg_);
  ui_rp_ = std::make_shared<UiRenderPass>(vkc_, vkg_);
  vkg_->RegisterRenderPass(scene_rp_);
  vkg_->RegisterRenderPass(ui_rp_);

  auto mp = std::make_shared<gui::details::MeshPipeline>(vkc, vkg, scene_rp_);
  scene_rp_->AddPipeline(mp);
  scene_pipelines_.push_back(mp);

  auto w = ecs::Rc<UiWindows>{}.MakeValid();
  w->callbacks_.insert({"GuisysDefault", default_gui_render});
  setup_default_keymap();
}

// TODO: Impl
GuiSystem::~GuiSystem() = default;

void GuiSystem::TickLogic() {
  // TODO: Foreach Simplical Render Config.
  glfwPollEvents();
  ProcessInputs();
}

void GuiSystem::ProcessInputs() {
  // detect key press...
  auto km = ecs::Rc<GuisysKeymap>{}.MakeValid();
  for (auto &[k, f] : km->keymap_) {
    if (glfwGetKey(win_->GetWindow(), k)) {
      f();
    }
  }
}

void GuiSystem::TickRender() {
  for (auto [ent, data] : ecs::ComponentManager<SimplicalRenderData>()) {
    if (!data->flush_ && data->lazy_load_) {
      continue;
    }
    // Get the component. and fill in all the required buffers.
    // 1. Fill the vertex buffer.
    CreateSceneSharedData(ent, data);
    // 2. Fill each pipeline's data
    for (auto &ppl : scene_pipelines_) {
      ppl->CreateEntityRenderData(ent, sbuffer_);
    }
  }
  // Make sure the data on the staging buffer is sent.
  sbuffer_.Flush();
  for (auto &ppl : scene_pipelines_) {
    ppl->UpdateUniformBuffer();
  }

  // Do Actual Rendering
  glfwPollEvents();
  auto rs = vkg_->Render();
  if (rs == VkRenderResult::kFailed) {
    throw std::runtime_error("vkgraphics render failed.");
  } else if (rs == VkRenderResult::kRecreateSwapchain) {
    AXES_INFO("Recreate Swapchain.");
    // TODO: Recreeate all the resources.
    auto cam = ecs::Rc<SceneCamera>{}.MakeValid();
    vkg_->RecreateSwapchain();
    Real width = static_cast<Real>(vkg_->GetSwapchainExtent().width);
    Real height = static_cast<Real>(vkg_->GetSwapchainExtent().height);
    cam->aspect_ = width / height;
    cam->update_ = true;

    cam.Publish();
  }
}

void GuiSystem::CreateSceneSharedData(ecs::EntityID ent,
                                      SimplicalRenderData *data) {
  ecs::ComponentManager<details::SceneRenderDataSharedGpu> sr;

  // 1. For shared part, i.e. vertex buffer, fill in directly.
  auto *shared = sr.AttachOrGet(ent, vkc_);
  size_t vertex_count = data->vertices_.Size();
  if (vertex_count == 0) {
    throw std::runtime_error("Error: No Vertex Provided for entity "
                             + std::to_string(ent));
  }
  vk::BufferCreateInfo bci;
  bci.setUsage(vk::BufferUsageFlagBits::eTransferDst
               | vk::BufferUsageFlagBits::eVertexBuffer)
      .setSharingMode(vk::SharingMode::eExclusive)
      .setSize(sizeof(details::SceneVertex) * vertex_count);

  VmaAllocationCreateInfo vaci;
  memset(&vaci, 0, sizeof(vaci));
  vaci.usage = VMA_MEMORY_USAGE_AUTO;
  vaci.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
  vaci.priority = 1.0f;
  vkc_->PrepareBuffer(shared->vertex_buffer_, bci, vaci);
  AXES_TRACE("GuiSystem: Create Shared Data for Entity {}: Vertex count = {}.", ent,
             data->vertices_.Size());
  sbuffer_.CopyBuffer(shared->vertex_buffer_, data->vertices_.Ptr(),
                      sizeof(details::SceneVertex) * data->vertices_.Size());
}

}  // namespace axes::gui
