#include <vma/vk_mem_alloc.h>

#include <axes/gui/details/glfwin.hpp>
#include <axes/gui/details/scene_pass.hpp>
#include <axes/gui/details/ui_pass.hpp>
#include <axes/gui/details/vkcontext.hpp>
#include <axes/gui/details/vkgraphics.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <iostream>

#include "axes/core/ecs/ecs.hpp"
#include "axes/core/init.hpp"
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/details/mesh_pipeline.hpp"
#include "axes/gui/tags.hpp"

using namespace axes;

int main() {
  init();
  // gui::SimplicalRenderData data;
  // data.vertices_.Resize(3);
  // data.vertices_[0].position_ = glm::vec3{-.5, .5, 0};
  // data.vertices_[1].position_ = glm::vec3{.5, .5, 0};
  // data.vertices_[2].position_ = glm::vec3{0, -.5, 0};
  // data.vertices_[0].color_ = glm::vec4(1);
  // data.vertices_[1].color_ = glm::vec4(1);
  // data.vertices_[2].color_ = glm::vec4(1);
  //
  // data.faces_.Resize(3);
  // data.faces_[0] = 0;
  // data.faces_[1] = 1;
  // data.faces_[2] = 2;
  //
  // data.color_bias_ = glm::vec4(0);
  // data.flush_ = true;
  //
  ecs::World world;
  ecs::EntityManager em{world.CreateEntity()};
  // em.Attach<gui::SimplicalRenderData>(data);

  {
    gui::WindowCreateInfo wci;
    auto win = std::make_shared<gui::GlfwWindow>(wci);
    gui::VkContextCreateInfo ci;
    ci.glfw_window_ = win.get();
    auto vkc = std::make_shared<gui::VkContext>(ci);

    // Vertex buffer.
    vk::BufferCreateInfo bci;
    bci.setSize(sizeof(gui::details::SceneVertex) * 3)
        .setUsage(vk::BufferUsageFlagBits::eVertexBuffer)
        .setSharingMode(vk::SharingMode::eExclusive);
    VmaAllocationCreateInfo vaci;
    memset(&vaci, 0, sizeof(vaci));
    vaci.usage = VMA_MEMORY_USAGE_AUTO;
    vaci.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT
                 | VMA_ALLOCATION_CREATE_MAPPED_BIT;
    vaci.priority = 1.0f;
    auto vertex_buffer = vkc->AllocateBuffer(bci, vaci);
    auto *vert_arr = static_cast<gui::details::SceneVertex *>(
        vertex_buffer.alloc_info_.pMappedData);

    // Index buffer
    bci.setUsage(vk::BufferUsageFlagBits::eIndexBuffer)
        .setSize(sizeof(uint32_t) * 3);
    auto index_buffer = vkc->AllocateBuffer(bci, vaci);
    auto *index_arr
        = static_cast<uint32_t *>(index_buffer.alloc_info_.pMappedData);

    // Instance buffer
    bci.setUsage(vk::BufferUsageFlagBits::eVertexBuffer)
        .setSize(sizeof(gui::details::MeshInstance));
    auto instance_buffer = vkc->AllocateBuffer(bci, vaci);
    auto *inst_arr = static_cast<gui::details::MeshInstance *>(
        instance_buffer.alloc_info_.pMappedData);

    vert_arr[0].color_ = glm::vec4(1, 0, 0, 1);
    vert_arr[1].color_ = glm::vec4(0, 1, 0, 1);
    vert_arr[2].color_ = glm::vec4(0, 0, 1, 1);

    vert_arr[0].position_ = glm::vec3(0.5, 0.5, 0);
    vert_arr[1].position_ = glm::vec3(-0.5, 0.5, 0);
    vert_arr[2].position_ = glm::vec3(0, -0.5, 0);

    index_arr[0] = 0;
    index_arr[1] = 1;
    index_arr[2] = 2;

    inst_arr[0].color_ = glm::vec4(0);
    inst_arr[0].position_ = glm::vec4(0);
    inst_arr[0].rotation_ = glm::vec4(0);

    auto *prender = em.Attach<gui::details::MeshRenderDataGpu>(vkc);
    prender->index_buffer_ = index_buffer;
    prender->instance_buffer_ = instance_buffer;
    prender->instance_count_ = 1;
    prender->index_count_ = 3;
    prender->is_valid_ = true;
    prender->pc_.model_ = glm::identity<glm::mat4>();

    auto *pshared = em.Attach<gui::details::SceneRenderDataSharedGpu>(vkc);
    pshared->vertex_buffer_ = vertex_buffer;

    auto vkg = std::make_shared<gui::VkGraphicsContext>(vkc, win);
    auto ui = std::make_shared<gui::UiRenderPass>(vkc, vkg);
    auto sc = std::make_shared<gui::SceneRenderPass>(vkc, vkg);
    vkg->RegisterRenderPass(sc);
    vkg->RegisterRenderPass(ui);

    auto mp = std::make_unique<gui::details::MeshPipeline>(vkc, vkg, sc);
    sc->AddPipeline(std::move(mp));
    while (!win->ShouldClose()) {
      glfwPollEvents();
      auto pos = win->GetCursurPosition();
      auto result = vkg->Render();
      if (result == gui::VkRenderResult::kRecreateSwapchain) {
        std::cout << "Recreate Swapchain" << std::endl;
        vkg->RecreateSwapchain();
      }
      if (win->IsKeyPressed(GLFW_KEY_Q)) {
        std::cout << pos.transpose() << std::endl;
      }
    }

    vkc->GetDevice().waitIdle();
  }
  shutdown();
  return 0;
}
