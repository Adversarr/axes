#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/timestepper/quasi_newton.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/io.hpp"
#include "ax/math/linsys/sparse/Cholmod.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/iota.hpp"
#include "ax/utils/time.hpp"

ABSL_FLAG(std::string, resolution, "low", "low, mid, high");
ABSL_FLAG(int, prolongation, 1, "Prolongation level");
ABSL_FLAG(std::string, strategy, "kReservedForExperimental", "");

using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;
int prolongation;

std::unique_ptr<fem::Timestepper_QuasiNewton<3>> ts;

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
  ts->GetElasticity().Update(ts->GetMesh()->GetVertices(), fem::ElasticityUpdateLevel::kEnergy);
  ts->GetElasticity().UpdateEnergy();
  ts->GetElasticity().GatherEnergyToVertices();
  auto e_per_vert = ts->GetElasticity().GetEnergyOnVertices();

  static real m = 0, M = 0;
  m = e_per_vert.minCoeff();
  M = e_per_vert.maxCoeff();

  gl::Colormap cmap(m, M);
  mesh.colors_.topRows<3>() = cmap(e_per_vert);
}

static bool running = false;
float dt = 1e-2;
math::vecxr fps;
bool disable_original;

std::vector<math::vecxr> uk, gk, sk, inertia;
std::unique_ptr<math::SparseSolverBase> solver;

void setup_spai();

void ui_callback(gl::UiRenderEvent) {
  static idx frame = 0;
  ImGui::Begin("FEM", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Checkbox("Running", &running);
  ImGui::Text("dt=%lf", dt);
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh()->GetNumElements(),
              ts->GetMesh()->GetNumVertices());
  if (ImGui::Button("Step") || running) {
    const auto& vert = ts->GetMesh()->GetVertices();
    auto time_start = ax::utils::GetCurrentTimeNanos();
    for (auto i : utils::iota(input_mesh.vertices_.cols())) {
      math::vec3r position = input_mesh.vertices_.col(i);
      if (position.x()> 4.9) {
        position.x() += dt * frame; 
        ts->GetMesh()->MarkDirichletBoundary(i, 0, position.x());
        ts->GetMesh()->MarkDirichletBoundary(i, 1, position.y());
        ts->GetMesh()->MarkDirichletBoundary(i, 2, position.z());
      }
    }
    ts->SetExternalAccelerationUniform(math::vec3r{0, -9.8, 0});


    ts->BeginTimestep();
    setup_spai();
    ts->SolveTimestep();
    ts->EndTimestep();
    auto time_end = ax::utils::GetCurrentTimeNanos();
    auto time_elapsed = (time_end - time_start) * 1e-9;
    fps[frame++ % fps.size()] = 1.0 / time_elapsed;
    std::cout << frame << " Dt=" << time_elapsed
              << "s, FPS=" << fps.sum() / std::min<idx>(100, frame) << std::endl;
    update_rendering();
  }
  ImGui::End();
}

void fix_negative_volume(math::field4i& tets, math::field3r const& verts) {
  for (auto i : utils::iota(tets.cols())) {
    auto a = verts.col(tets(0, i));
    auto b = verts.col(tets(1, i));
    auto c = verts.col(tets(2, i));
    auto d = verts.col(tets(3, i));
    math::mat4r tet;
    tet << a, b, c, d, 0, 0, 0, 1;
    if (math::det(tet) < 0) {
      std::swap(tets(1, i), tets(2, i));
    }
  }
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  fps.setZero(100);

  std::string resolution = absl::GetFlag(FLAGS_resolution);
  std::string tet_file, vet_file;
  tet_file = utils::get_asset("/mesh/npy/beam_" + resolution + "_res_elements.npy");
  vet_file = utils::get_asset("/mesh/npy/beam_" + resolution + "_res_vertices.npy");

  auto tet = math::read_npy_v10_idx(tet_file);
  auto vet = math::read_npy_v10_real(vet_file);
  input_mesh.indices_ = tet.transpose();
  input_mesh.vertices_ = vet.transpose();

  ts = std::make_unique<fem::Timestepper_QuasiNewton<3>>(std::make_shared<fem::TriMesh<3>>());
  std::string strategy = absl::GetFlag(FLAGS_strategy);
  ts->SetOptions({{"lbfgs_strategy", strategy}, {"record_trajectory", true}});
  ts->SetYoungs(1e7);
  ts->SetPoissonRatio(0.45);
  ts->GetMesh()->SetMesh(input_mesh.indices_, input_mesh.vertices_);
  ts->GetMesh()->SetNumDofPerVertex(3);
  ts->SetDensity(1e3);
  ts->SetupElasticity("stable_neohookean", "cpu");
  AX_CHECK_OK(ts->Initialize());

  /************************* SECT: Setup Boundaries *************************/
  for (auto i : utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (abs(position.x())> 4.9) {
      ts->GetMesh()->MarkDirichletBoundary(i, 0, position.x());
      ts->GetMesh()->MarkDirichletBoundary(i, 1, position.y());
      ts->GetMesh()->MarkDirichletBoundary(i, 2, position.z());
    }
  }
  ts->SetExternalAccelerationUniform(math::vec3r{0, -9.8, 0});

  std::cout << "Running Parameters: " << ts->GetOptions() << std::endl;

  auto edges = geo::get_edges(input_mesh.indices_);
  printf("Edge set size=%ld\n", edges.cols());

  prolongation = absl::GetFlag(FLAGS_prolongation);
  ts->BeginSimulation(dt);
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

void setup_spai() {
  // 1. Naive strategy, compute the true inverse, and set to the solver.
  math::spmatr H = ts->Hessian(ts->GetInitialGuess() + ts->GetDisplacement(), true);

  math::SparseSolver_Cholmod chol;
  chol.SetProblem(H).Compute();
  math::matxxr H_inv = chol.Inverse();

  auto& sia = ensure_resource<fem::SparseInverseApproximator>();
  if (sia.A_.nonZeros() == 0) {
    // Empty, set according to prolongation.
    math::spmatr M = ts->GetMassMatrix();
    math::spmatr_for_each(M, [](idx, idx, real& value) { value = 1.0; });
    for (int i = 0; i < prolongation; ++i) {
      M = M * M;
      M.makeCompressed();
      math::spmatr_for_each(M, [](idx, idx, real& value) { value = 1.0; });
    }
    sia.A_ = M;
  }

  // set the sparse fill-ins.
  math::spmatr_for_each(sia.A_, [&H_inv](idx i, idx j, real& value) { 
      value = H_inv(i, j);
  });
}
