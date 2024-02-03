#include <absl/flags/declare.h>
#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/window.hpp"

ABSL_FLAG(bool, echo, false, "Echo events");

using namespace ax;

struct Echo {
  void WSize(const gl::WindowSizeEvent& evt) const {
    LOG(INFO) << "Window size: " << math::transpose(evt.size_);
  }

  void WPos(const gl::WindowPosEvent& evt) const {
    LOG(INFO) << "Window pos: " << math::transpose(evt.pos_);
  }

  void Key(const gl::KeyboardEvent& evt) const {
    LOG(INFO) << "Key: " << evt.key_ << std::endl << "Action: " << evt.action_;
  }
};

gl::Lines create_dummy() {
  gl::Lines lines;
  lines.vertices_.resize(3, 3);
  lines.vertices_.col(0) = math::vec3r::Zero();
  lines.vertices_.col(1) = math::vec3r{0.5, 0.5, 0.};
  lines.vertices_.col(2) = math::vec3r{0.5, -0.5, 0.};
  lines.colors_.resize(4, 3);
  lines.colors_.setRandom();
  lines.colors_ = lines.colors_ * 0.5;
  lines.colors_.colwise() += math::ones<4>();
  lines.indices_.resize(2, 2);
  lines.indices_.col(0) = math::vec2i{0, 1};
  lines.indices_.col(1) = math::vec2i{1, 2};
  lines.flush_ = true;
  return lines;
}

int main(int argc, char** argv) {
  ax::init(argc, argv);
  auto& ctx = add_resource<gl::Context>();

  if (absl::GetFlag(FLAGS_echo)) {
    auto& echo = add_resource<Echo>();
    connect<gl::WindowSizeEvent, &Echo::WSize>(echo);
    connect<gl::WindowPosEvent, &Echo::WPos>(echo);
    connect<gl::KeyboardEvent, &Echo::Key>(echo);
  }

  auto ent = create_entity();
  auto& line = add_component<gl::Lines>(ent, create_dummy());
  /****************************** Main Loop ******************************/
  auto& win = ctx.GetWindow();
  time_t start = time(nullptr);
  while (!win.ShouldClose()) {
    CHECK_OK(ctx.TickLogic());
    CHECK_OK(ctx.TickRender());

    time_t end = time(nullptr);
    double delta = difftime(end, start);
    if (delta > 0.1) {
      line = create_dummy();
      start = end;
    }
  }

  erase_resource<gl::Context>();
  clean_up();
  return 0;
}
