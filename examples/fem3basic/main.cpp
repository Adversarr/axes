#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/deform.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/mesh/p1mesh.hpp"
#include "ax/fem/timestepper.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/iota.hpp"

ABSL_FLAG(std::string, input, "plane.obj", "Input 2D Mesh.");
ABSL_FLAG(int, N, 5, "Num of division.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");

using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;

UPtr<fem::TimeStepperBase<3>> ts;


void update_rendering() {
  auto &mesh = get_component<gl::Mesh>(out);
  auto sm = ts->GetMesh().ExtractSurface();
  mesh.vertices_ = sm.vertices_;
  mesh.indices_ = sm.indices_;
  mesh.colors_.setOnes(4, mesh.vertices_.cols());
  mesh.flush_ = true;
  auto &lines = get_component<gl::Lines>(out);
  lines = gl::Lines::Create(mesh);
  lines.colors_.topRows<3>().setZero();

  ts->GetElasticity().UpdateDeformationGradient();
  auto e_per_elem = ts->GetElasticity().Energy(lame);
  auto e_per_vert = ts->GetDeformation().EnergyToVertices(e_per_elem);
  static real m = 0, M = 0;
  m = std::min(m, e_per_vert.minCoeff());
  M = std::max(M, e_per_vert.maxCoeff());

  gl::Colormap cmap(m, M);
  mesh.colors_.topRows<3>() = cmap(e_per_vert);
}

static bool running = false;
void ui_callback(gl::UiRenderEvent ) {
  ImGui::Begin("FEM");
  ImGui::Checkbox("Running", &running);
  if (ImGui::Button("Step") || running) {
    AX_CHECK_OK(ts->Step(1e-3));
    update_rendering();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  lame = fem::elasticity::compute_lame(1e5, 0.33);
  int nx = absl::GetFlag(FLAGS_N);
  input_mesh = geo::tet_cube(0.5, 2 * nx, nx, nx);
  input_mesh.vertices_.row(0) *= 2;
  ts = std::make_unique<fem::TimeStepperBase<3>>(std::make_unique<fem::P1Mesh<3>>());
  ts->SetLame(lame);
  AX_CHECK_OK(ts->GetMesh().SetMesh(input_mesh.indices_, input_mesh.vertices_));
  for (auto i: utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (math::abs(position.x()) > 1.0 - 1.0 / nx) {
      // Mark as dirichlet bc.
      ts->GetMesh().MarkDirichletBoundary(i, 0, position.x());
      ts->GetMesh().MarkDirichletBoundary(i, 1, position.y());
      ts->GetMesh().MarkDirichletBoundary(i, 2, position.z());
    }
  }
  AX_CHECK_OK(ts->Init());
  ts->SetDensity(3);
  out = create_entity();
  add_component<gl::Mesh>(out);
  add_component<gl::Lines>(out);
  update_rendering();
  connect<gl::UiRenderEvent, &ui_callback>();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
