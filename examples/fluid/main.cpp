#include <absl/flags/flag.h>
#include <imgui.h>
#include <implot.h>

#include "axes/core/echo.hpp"
#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/io.hpp"
#include "axes/geometry/normal.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/primitives/quiver.hpp"
#include "axes/gl/utils.hpp"
#include "axes/utils/asset.hpp"
using namespace ax;

#include "axes/pde/advect/lattice.hpp"


math::Lattice<2, real> u;

math::matxxr u_mat;

void ui_callback(gl::UiRenderEvent) {
  ImGui::Text("Hello, world!");

  // u_mat = u.X().ToMatrix();

  if (ImGui::Button("Advect")) {
    auto advection = pde::AdvectionProblem<2>(0.1, 1.0);
    // auto result = advection.AdvectVelocity(false);
    // if (result.ok()) {
    //   u = result.value();
    // } else {
    //   AX_LOG(ERROR) << "Failed to advect velocity field: " << result.status();
    // }
  }

  if (ImPlot::BeginPlot("Velocity Field")) {
    ImPlot::PlotHeatmap("X", u_mat.data(), u_mat.rows(), u_mat.cols());
    ImPlot::EndPlot();
  }
}


int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  // TODO: Implement.
  using namespace math;
  u.Reshape({10, 10});
  auto advection = pde::AdvectionProblem<2>(0.1, 1.0);


  clean_up();
  return 0;
}
