#include <absl/flags/flag.h>
#include <imgui.h>
#include <implot.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/utils.hpp"
using namespace ax;

#include "axes/pde/advect/lattice.hpp"


math::Lattice<2, math::vec2r> u, g;

math::matxxr u_mat;
math::vecxr u_x_0;

idx N = 50;

void ui_callback(gl::UiRenderEvent) {
  ImGui::Text("Hello, world!");

  u_mat.resize(N, N);
  u_x_0.resize(N);
  for (auto [i, j] : utils::multi_iota(N, N)) {
    u_mat(i, j) = u(i, j).x();
  }
  for (idx i = 0; i < N; ++i) {
    u_x_0[i] = u(i, 0).x();
  }
  static bool running = false;
  ImGui::Checkbox("Running", &running);
  if (running) {
    auto x = pde::bfecc<2>(g, u, 1, true);
    auto uback = u;
    for (auto [i, j] : utils::multi_iota(N, N)) {
      u(i, j) = math::lerp_outside<2>(uback, x(i, j), true);
    }
  }

  if (ImPlot::BeginPlot("Velocity Field")) {
    // ImPlot::PlotHeatmap("X", u_mat.data(), u_mat.rows(), u_mat.cols());
    ImPlot::PlotLine("X0", u_x_0.data(), u_x_0.size());
    ImPlot::EndPlot();
  }
}


int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  // TODO: Implement.

  connect<gl::UiRenderEvent, &ui_callback>();
  using namespace math;
  u.Reshape({N, N});
  g.Reshape({N, N});

  for (auto [i, j] : utils::multi_iota(N, N)) {
    u(i, j) = vec2r{i * (N - i) / real(N * N) + 0.1, 0};
    g(i, j) = vec2r{i, j};
  }

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
