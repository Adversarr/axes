#include <vma/vk_mem_alloc.h>

#include <axes/gui/details/glfwin.hpp>
#include <axes/gui/details/guisys.hpp>
#include <axes/gui/details/scene_pass.hpp>
#include <axes/gui/details/ui_pass.hpp>
#include <axes/gui/details/vkcontext.hpp>
#include <axes/gui/details/vkgraphics.hpp>
#include <backward.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <iostream>

#include "axes/core/ecs/ecs.hpp"
#include "axes/core/init.hpp"
#include "axes/core/systems/ecsinfo.hpp"
#include "axes/core/utils/log.hpp"
#include "axes/gui/details/buffers.hpp"
#include "axes/gui/tags.hpp"

using namespace axes;

int main() {
  init();
  // axes::utils::set_default_log_level(spdlog::level::trace);
  gui::SimplicalRenderData data;
  data.vertices_.Resize(3);
  data.vertices_[0].position_ = glm::vec3{1, 0, 0};
  data.vertices_[1].position_ = glm::vec3{0, 1, 0};
  data.vertices_[2].position_ = glm::vec3{0, 0, 1};
  data.vertices_[0].color_ = glm::vec4(.7, 0, 0, 1);
  data.vertices_[1].color_ = glm::vec4(0, .7, 0, 1);
  data.vertices_[2].color_ = glm::vec4(0, 0, .7, 1);
  data.vertices_[0].normal_ = glm::vec3(1, 0, 0);
  data.vertices_[1].normal_ = glm::vec3(0, 1, 0);
  data.vertices_[2].normal_ = glm::vec3(0, 0, 1);

  data.faces_.Resize(3);
  data.faces_[0] = 0;
  data.faces_[1] = 1;
  data.faces_[2] = 2;

  data.instances_.Resize(1);
  data.instances_[0].position_ = glm::vec3(0);
  data.instances_[0].rotation_ = glm::vec4(0);

  data.color_bias_ = glm::vec4(0);
  data.flush_ = true;

  ecs::World world;
  ecs::EntityManager em{world.CreateEntity()};
  EcsInfoSystem ei;
  em.Attach<gui::SimplicalRenderData>(data);

  {
    gui::WindowCreateInfo wci;
    auto win = std::make_shared<gui::GlfwWindow>(wci);
    gui::VkContextCreateInfo ci;
    ci.glfw_window_ = win.get();
    auto vkc = std::make_shared<gui::VkContext>(ci);
    auto vkg = std::make_shared<gui::VkGraphicsContext>(vkc, win);
    gui::GuiSystem uisys(win, vkc, vkg);
    while (!win->ShouldClose()) {
      uisys.TickLogic();
      uisys.TickRender();
    }
    vkc->GetDevice().waitIdle();
  }
  shutdown();
  return 0;
}
