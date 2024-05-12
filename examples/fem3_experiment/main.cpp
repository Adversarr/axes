#include <imgui.h>
#include <implot.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/elasticity_gpu.cuh"
#include "ax/fem/timestepper.hpp"
#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/geometry/io.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/io.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/iota.hpp"
#include "ax/utils/time.hpp"

ABSL_FLAG(std::string, input, "plane.obj", "Input 2D Mesh.");
ABSL_FLAG(int, N, 2, "Num of division.");
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
  auto& mesh = get_component<gl::Mesh>(out);
  if (mesh.indices_.size() == 0) {
    mesh.indices_ = geo::get_boundary_triangles(input_mesh.vertices_, input_mesh.indices_);
  }
  mesh.vertices_ = ts->GetPosition();
  mesh.colors_.setOnes(4, mesh.vertices_.cols());
  mesh.flush_ = true;
  mesh.use_lighting_ = false;
  auto& lines = get_component<gl::Lines>(out);
  if (lines.indices_.size() == 0) {
    lines = gl::Lines::Create(mesh);
  }
  lines.vertices_ = mesh.vertices_;
  lines.flush_ = true;
  lines.colors_.topRows<3>().setZero();

  ts->GetElasticity().Update(ts->GetMesh().GetVertices(), fem::ElasticityUpdateLevel::kEnergy);
  auto e_per_elem = ts->GetElasticity().Energy(lame);
  auto e_per_vert = ts->GetElasticity().GatherEnergy(e_per_elem);
  static real m = 0, M = 0;
  m = e_per_vert.minCoeff();
  M = e_per_vert.maxCoeff();

  gl::Colormap cmap(m, M);
  mesh.colors_.topRows<3>() = cmap(e_per_vert);
}

static bool running = false;
float dt = 1e-2;
math::vecxr fps;
math::vecxr dx_eig, eval, linear_fit;

bool plot_log = false;
void ui_callback(gl::UiRenderEvent) {
  ImGui::Begin("FEM");
  ImGui::Checkbox("Running", &running);
  ImGui::InputFloat("dt", &dt);
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh().GetNumElements(),
              ts->GetMesh().GetNumVertices());
  ImGui::Checkbox("Log", &plot_log);
  if (ImGui::Button("Step") || running) {
    ts->BeginTimestep(dt);
    math::field3r u0 = ts->GetInitialGuess();
    ts->SolveTimestep();
    math::field3r u1 = ts->GetSolution();
    ts->EndTimestep();

    math::vecxr dx = (u1 - u0).reshaped();
    ts->GetMesh().FilterVector(dx, true);
    // stiffness:
    math::sp_matxxr K = ts->GetStiffnessMatrix(ts->GetPosition(), true);
    math::sp_matxxr M = ts->GetMassMatrix();

    // math::field3r rot_field = vert;
    // rot_field.row(0).setOnes();
    // rot_field.row(1).setZero();
    // rot_field.row(2).setZero();
    // std::cout << rot_field.reshaped().dot(K * rot_field.reshaped()) << std::endl;

    // ts->GetMesh().FilterMatrixFull(K);
    // ts->GetMesh().FilterMatrixFull(M);
    // find the eigen values of K
    auto [vec, val] = math::eig(K.toDense());
    // Eigen::GeneralizedSelfAdjointEigenSolver<math::matxxr> es(K, M);
    // auto vec = es.eigenvectors();
    // auto val = es.eigenvalues();
    // Decompose dx into the eigenvectors of K
    dx_eig = vec.transpose() * dx;
    // // Scaling.
    for (auto i : utils::iota(dx_eig.size())) {
      dx_eig(i) /= vec.col(i).norm();
      // dx_eig(i) /= val(i) + 1e-6;
      dx_eig(i) = std::abs(dx_eig(i));
    }

    eval = val * dt * dt;
    eval /= eval.maxCoeff();
    dx_eig /= dx_eig.maxCoeff();
    if (plot_log) {
      for (auto i : utils::iota(dx_eig.size())) {
        dx_eig(i) = std::log10(dx_eig(i) + 1e-6) + 6;
      }
      for (auto i : utils::iota(eval.size())) {
        eval(i) = std::log10(eval(i) + 1e-6) + 6;
      }
    }
    update_rendering();

    if (scene == SCENE_TWIST) {
      // Apply some Dirichlet BC
      math::mat3r rotate = Eigen::AngleAxis<real>(dt, math::vec3r::UnitX()).matrix();
      u0 = ts->GetPosition();
      for (auto i : utils::iota(u0.cols())) {
        const auto& position = u0.col(i);
        if (-position.x() > 1.9) {
          // Mark as dirichlet bc.
          math::vec3r p = rotate * position;
          ts->GetMesh().MarkDirichletBoundary(i, 0, p.x());
          ts->GetMesh().MarkDirichletBoundary(i, 1, p.y());
          ts->GetMesh().MarkDirichletBoundary(i, 2, p.z());
        }
      }
    }
  }

  if (ImPlot::BeginPlot("eig")) {
    ImPlot::PlotLine("dx_eig", dx_eig.data(), dx_eig.size());
    ImPlot::PlotLine("eval", eval.data(), eval.size());
    // fit the curve dx_eig by n.
    math::matxxr a(dx_eig.size(), 3);
    a.col(0).setLinSpaced(0, 1);
    a.col(1) = a.col(0).array() * a.col(0).array();
    a.col(2).setOnes();
    math::mat3r ata = a.transpose() * a;
    math::vec3r atb = a.transpose() * dx_eig;
    auto qr = ata.householderQr();
    math::vec3r c = qr.solve(atb);
    linear_fit = c[0] * a.col(0) + c[1] * a.col(1) + c[2] * a.col(2);
    ImPlot::PlotLine("fit", linear_fit.data(), linear_fit.size());
    std::cout << "Slope and Bias: " << c.transpose() << std::endl;
    ImPlot::EndPlot();
  }

  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  fps.setZero(100);
  scene = absl::GetFlag(FLAGS_scene);
  lame = fem::elasticity::compute_lame(1e7, 0.45);
  nx = absl::GetFlag(FLAGS_N);

  auto cube = geo::tet_cube(0.5, nx * 4, nx, nx);
  input_mesh.vertices_ = std::move(cube.vertices_);
  input_mesh.indices_ = std::move(cube.indices_);
  input_mesh.vertices_.row(0) *= 4;
  // std::string tet_file = utils::get_asset("/mesh/npy/beam_high_res_elements.npy"),
  //             vet_file = utils::get_asset("/mesh/npy/beam_high_res_vertices.npy");
  // auto tet = math::read_npy_v10_idx(tet_file);
  // auto vet = math::read_npy_v10_real(vet_file);
  // input_mesh.indices_ = tet->transpose();
  // input_mesh.vertices_ = vet->transpose();

  ts = std::make_unique<fem::Timestepper_NaiveOptim<3>>(std::make_unique<fem::TriMesh<3>>());
  ts->SetLame(lame);
  AX_CHECK_OK(ts->GetMesh().SetMesh(input_mesh.indices_, input_mesh.vertices_));
  for (auto i : utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (abs(position.x()) > 1.9) {
      // Mark as dirichlet bc.
      if (scene == SCENE_TWIST || position.x() > 1.9) {
        ts->GetMesh().MarkDirichletBoundary(i, 0, position.x());
        ts->GetMesh().MarkDirichletBoundary(i, 1, position.y());
        ts->GetMesh().MarkDirichletBoundary(i, 2, position.z());
      }
    }
  }

  AX_CHECK_OK(ts->Initialize());
  ts->SetupElasticity("stable_neohookean", "cpu");
  ts->SetDensity(1e3);
  ts->BeginSimulation();

  ts->SetExternalAccelerationUniform(math::vec3r(0, -9.8, 0));

  out = create_entity();
  add_component<gl::Mesh>(out);
  add_component<gl::Lines>(out);
  update_rendering();
  connect<gl::UiRenderEvent, &ui_callback>();
  AX_CHECK_OK(gl::enter_main_loop());
  ts.reset();
  clean_up();
  return 0;
}
