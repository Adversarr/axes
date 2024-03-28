#include <imgui.h>

#include "axes/core/entt.hpp"
#include "axes/core/init.hpp"
#include "axes/geometry/io.hpp"
#include "axes/geometry/primitives.hpp"
#include "axes/gl/colormap.hpp"
#include "axes/gl/context.hpp"
#include "axes/gl/primitives/lines.hpp"
#include "axes/gl/primitives/mesh.hpp"
#include "axes/gl/utils.hpp"
#include "axes/pde/elasticity/neohookean_bw.hpp"
#include "axes/pde/elasticity/stvk.hpp"
#include "axes/pde/elasticity/linear.hpp"
#include "axes/pde/fem/deform.hpp"
#include "axes/pde/fem/elasticity.hpp"
#include "axes/pde/fem/p1mesh.hpp"
#include "axes/pde/fem/timestepper.hpp"
#include "axes/utils/asset.hpp"
#include "axes/utils/iota.hpp"

ABSL_FLAG(std::string, input, "plane.obj", "Input 2D Mesh.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");

using namespace ax;
Entity out;
geo::SurfaceMesh input_mesh;
math::vec2r lame;

UPtr<pde::fem::TimeStepperBase<2>> ts;


void update_rendering() {
  auto &mesh = get_component<gl::Mesh>(out);
  mesh.vertices_.resize(3, input_mesh.vertices_.cols());
  mesh.vertices_.topRows<2>() = ts->GetMesh().GetVertices();
  mesh.vertices_.row(2).setZero();
  mesh.indices_ = input_mesh.indices_;
  mesh.colors_.setOnes(4, mesh.vertices_.cols());
  mesh.flush_ = true;
  auto &lines = get_component<gl::Lines>(out);
  if (lines .vertices_.cols() == 0) {
    lines = gl::Lines::Create(mesh);
    lines.colors_.topRows<3>().setZero();
  } else {
    lines.vertices_ = mesh.vertices_;
    lines.flush_ = true;
  }

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
    AX_CHECK_OK(ts->Step(3e-4));
    update_rendering();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  lame = pde::elasticity::compute_lame(1e5, 0.45);
  input_mesh = ax::geo::read_obj(ax::utils::get_asset("/mesh/obj/" + absl::GetFlag(FLAGS_input))).value();
  ts = std::make_unique<pde::fem::TimeStepperBase<2>>(std::make_unique<pde::fem::P1Mesh<2>>());
  AX_CHECK_OK(ts->GetMesh().SetMesh(input_mesh.indices_, input_mesh.vertices_.topRows<2>()));
  for (auto i: utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (position.y() > 0.3) {
      // Mark as dirichlet bc.
      ts->GetMesh().MarkDirichletBoundary(i, 0, position.x());
      ts->GetMesh().MarkDirichletBoundary(i, 1, position.y());
    }
  }
  AX_CHECK_OK(ts->Init());
  out = create_entity();
  add_component<gl::Mesh>(out);
  add_component<gl::Lines>(out);
  update_rendering();
  connect<gl::UiRenderEvent, &ui_callback>();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
