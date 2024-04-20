#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/deform.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/mesh/p1mesh.hpp"
#include "ax/fem/timestepper.hpp"
#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/io.hpp"
#include "ax/utils/iota.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/time.hpp"


ABSL_FLAG(std::string, input, "plane.obj", "Input 2D Mesh.");
ABSL_FLAG(int, N, 7, "Num of division.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");
ABSL_FLAG(bool, scene, 0, "id of scene, 0 for twist, 1 for bend.");
int nx;
using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;

#define SCENE_TWIST 0
#define SCENE_BEND 1
int scene;

UPtr<fem::TimeStepperBase<3>> ts;


void update_rendering() {
  auto &mesh = get_component<gl::Mesh>(out);
  if (mesh.indices_ .size() == 0) {
    mesh.indices_ = geo::get_boundary_triangles(input_mesh.vertices_, input_mesh.indices_);
  }
  mesh.vertices_ = ts->GetMesh().GetVertices();
  mesh.colors_.setOnes(4, mesh.vertices_.cols());
  mesh.flush_ = true;
  mesh.use_lighting_ = false;
  auto &lines = get_component<gl::Lines>(out);
  if (lines.indices_.size() == 0) {
    lines = gl::Lines::Create(mesh);
  }
  lines.vertices_ = mesh.vertices_;
  lines.flush_ = true;
  lines.colors_.topRows<3>().setZero();

  ts->GetElasticity().UpdateDeformationGradient();
  auto e_per_elem = ts->GetElasticity().Energy(lame);
  auto e_per_vert = ts->GetDeformation().EnergyToVertices(e_per_elem);
  static real m = 0, M = 0;
  m = e_per_vert.minCoeff();
  M = e_per_vert.maxCoeff();

  gl::Colormap cmap(m, M);
  mesh.colors_.topRows<3>() = cmap(e_per_vert);
}

static bool running = false;
float dt = 1e-3;
math::vecxr fps;
void ui_callback(gl::UiRenderEvent ) {
  ImGui::Begin("FEM", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Checkbox("Running", &running);
  ImGui::InputFloat("dt", &dt);
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh().GetNumElements(), ts->GetMesh().GetNumVertices());
  if (ImGui::Button("Step") || running) {
    const auto& vert = ts->GetMesh().GetVertices();
    if (scene == SCENE_TWIST) {
      // Apply some Dirichlet BC
      math::mat3r rotate = Eigen::AngleAxis<real>(dt, math::vec3r::UnitX()).matrix();
      for (auto i : utils::iota(vert.cols())) {
        const auto& position = vert.col(i);
        if (-position.x() > 4.9) {
          // Mark as dirichlet bc.
          math::vec3r p = rotate * position;
          ts->GetMesh().MarkDirichletBoundary(i, 0, p.x());
          ts->GetMesh().MarkDirichletBoundary(i, 1, p.y());
          ts->GetMesh().MarkDirichletBoundary(i, 2, p.z());
        }
      }
    }

    auto time_start = ax::utils::GetCurrentTimeNanos();
    static idx frame = 0;
    AX_CHECK_OK(ts->Step(dt));
    auto time_end = ax::utils::GetCurrentTimeNanos();
    auto time_elapsed = (time_end - time_start) * 1e-9;
    fps[frame++ % fps.size()] = 1.0 / time_elapsed;
    std::cout << frame << ": FPS=" << fps.sum() / std::min<idx>(100, frame) << std::endl;
    update_rendering();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  fps.setZero(100);
  scene = absl::GetFlag(FLAGS_scene);
  lame = fem::elasticity::compute_lame(1e7, 0.45);
  nx = absl::GetFlag(FLAGS_N);
  std::string tet_file = utils::get_asset("/mesh/npy/beam_mid_res_elements.npy"),
              vet_file = utils::get_asset("/mesh/npy/beam_mid_res_vertices.npy");
  auto tet = math::read_npy_v10_idx(tet_file);
  auto vet = math::read_npy_v10_real(vet_file);
  input_mesh.indices_ = tet->transpose();
  input_mesh.vertices_ = vet->transpose();

  ts = std::make_unique<fem::Timestepper_NaiveOptim<3>>(std::make_unique<fem::P1Mesh<3>>());
  ts->SetLame(lame);
  AX_CHECK_OK(ts->GetMesh().SetMesh(input_mesh.indices_, input_mesh.vertices_));
  for (auto i: utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (math::abs(position.x()) > 4.9) {
      // Mark as dirichlet bc.
      if (scene == SCENE_TWIST || position.x() > 4.9) {
        ts->GetMesh().MarkDirichletBoundary(i, 0, position.x());
        ts->GetMesh().MarkDirichletBoundary(i, 1, position.y());
        ts->GetMesh().MarkDirichletBoundary(i, 2, position.z());
      }
    }
  }
  AX_CHECK_OK(ts->Init());
  ts->SetupElasticity<fem::elasticity::IsotropicARAP>();
  ts->SetDensity(1e3);
  out = create_entity();
  add_component<gl::Mesh>(out);
  add_component<gl::Lines>(out);
  update_rendering();
  connect<gl::UiRenderEvent, &ui_callback>();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
