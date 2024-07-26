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
ABSL_FLAG(std::string, dest, "output", "output directory");
ABSL_FLAG(int, nrandomgk, 0, "Num of random gk per step");
ABSL_FLAG(bool, disable_original, false, "disable original");
ABSL_FLAG(int, seperation, 50, "seperation");
ABSL_FLAG(int, n_save_sp_inverse, 10, "n_save_sp_inverse");

using namespace ax;
int seperation, nrandomgk, n_save_sp_inverse;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;
std::string dest;

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

void put_into_data() {
  auto const& traj = ts->GetLastTrajectory();
  solver = math::SparseSolverBase::Create(math::SparseSolverKind::kCholmod);
  auto H = ts->Hessian(ts->GetDisplacement(), true);
  solver->SetProblem(H).Compute();
  math::field3r iner = traj.front();
  idx const ndof = iner.size();
  if (!disable_original) {
    for (auto const& u : traj) {
      math::vecxr g = ts->Gradient(u).reshaped();
      auto result = solver->Solve(g);
      if (g.dot(result.solution_) < 0) {
        continue;
      }
      if (result.solution_.norm() < 1e-6) {
        continue;
      }
      uk.push_back(u.reshaped());
      gk.push_back(g);
      sk.push_back(result.solution_);
      inertia.push_back(iner.reshaped());
    }
  }

  math::vecxr u = ts->GetDisplacement().reshaped();
  for (auto _ : utils::iota(static_cast<idx>(nrandomgk))) {
    math::vecxr g = math::vecxr::Random(ndof) * dt * 0.1;
    g *= ts->GetMesh()->GetDirichletBoundaryMask().reshaped();
    auto result = solver->Solve(g);
    if (g.allFinite() && result.solution_.allFinite()) {
      if (g.dot(result.solution_) < 0) {
        continue;
      }
      if (result.solution_.norm() < 1e-6) {
        continue;
      }
      uk.push_back(iner.reshaped());
      gk.push_back(g);
      sk.push_back(result.solution_);
      inertia.push_back(iner.reshaped());
    }
  }

  std::cout << uk.size() << std::endl;
}

void write_into_seps() {
  static int cnt = 0;
  if (uk.size() > static_cast<size_t>(seperation)) {
    idx n_dof = uk.front().size();
    math::matxxr X;
    X.resize(n_dof, seperation);
    for (idx i = 0; i < seperation; ++i) {
      X.col(i) = uk[i];
    }
    X.transposeInPlace();
    (math::write_npy_v10(dest + "/uk_" + std::to_string(cnt) + ".npy", X));

    X.resize(n_dof, seperation);
    for (idx i = 0; i < seperation; ++i) {
      X.col(i) = gk[i];
    }
    X.transposeInPlace();
    (math::write_npy_v10(dest + "/gk_" + std::to_string(cnt) + ".npy", X));

    X.resize(n_dof, seperation);
    for (idx i = 0; i < seperation; ++i) {
      X.col(i) = sk[i];
    }
    X.transposeInPlace();
    (math::write_npy_v10(dest + "/sk_" + std::to_string(cnt) + ".npy", X));

    X.resize(n_dof, seperation);
    for (idx i = 0; i < seperation; ++i) {
      X.col(i) = inertia[i];
    }
    X.transposeInPlace();
    (math::write_npy_v10(dest + "/inertia_" + std::to_string(cnt) + ".npy", X));

    uk.clear();
    gk.clear();
    sk.clear();
    inertia.clear();

    std::cout << "EXPORTED " << cnt << std::endl;
    cnt += 1;
  }
}

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

    ts->BeginTimestep();
    ts->SolveTimestep();
    ts->EndTimestep();
    auto time_end = ax::utils::GetCurrentTimeNanos();
    auto time_elapsed = (time_end - time_start) * 1e-9;
    fps[frame++ % fps.size()] = 1.0 / time_elapsed;
    std::cout << frame << " Dt=" << time_elapsed
              << "s, FPS=" << fps.sum() / std::min<idx>(100, frame) << std::endl;
    update_rendering();
    put_into_data();
    write_into_seps();
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
  ts->SetOptions({{"lbfgs_strategy", "kLaplacian"}, {"record_trajectory", true}});
  ts->SetYoungs(1e7);
  ts->SetPoissonRatio(0.45);
  ts->GetMesh()->SetMesh(input_mesh.indices_, input_mesh.vertices_);
  ts->GetMesh()->SetNumDofPerVertex(3);
  ts->SetDensity(1e3);
  ts->SetupElasticity("stable_neohookean", "gpu");
  AX_CHECK_OK(ts->Initialize());

  /************************* SECT: Setup Boundaries *************************/
  for (auto i : utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (position.x() > 4.9) {
      ts->GetMesh()->MarkDirichletBoundary(i, 0, position.x());
      ts->GetMesh()->MarkDirichletBoundary(i, 1, position.y());
      ts->GetMesh()->MarkDirichletBoundary(i, 2, position.z());
    }
  }
  ts->SetExternalAccelerationUniform(math::vec3r{0, -9.8, 0});

  std::cout << "Running Parameters: " << ts->GetOptions() << std::endl;
  dest = absl::GetFlag(FLAGS_dest);
  seperation = absl::GetFlag(FLAGS_seperation);
  nrandomgk = absl::GetFlag(FLAGS_nrandomgk);
  disable_original = absl::GetFlag(FLAGS_disable_original);
  n_save_sp_inverse = absl::GetFlag(FLAGS_n_save_sp_inverse);

  auto edges = geo::get_edges(input_mesh.indices_);
  printf("Edge set size=%ld\n", edges.cols());

  math::matxxr V = input_mesh.vertices_.transpose();
  math::matxxi E = input_mesh.indices_.transpose();
  (math::write_npy_v10(dest + "/vertices.npy", V));
  (math::write_npy_v10(dest + "/elements.npy", E));
  math::vecxr bc_mask = ts->GetMesh()->GetDirichletBoundaryMask().row(0).transpose();
  (math::write_npy_v10(dest + "/bc_masks.npy", bc_mask));
  math::vecxi node_type = (1 - bc_mask.array()).cast<idx>();
  (math::write_npy_v10(dest + "/node_types.npy", node_type));
  math::vecxr mass
      = ts->GetMassMatrixOriginal() * math::vecxr::Ones(ts->GetMesh()->GetNumVertices());
  (math::write_npy_v10(dest + "/mass.npy", mass));
  ts->BeginSimulation(dt);
  auto L = ts->GetLaplacianAsApproximation();

  {
    auto solver = math::SparseSolver_Cholmod();
    solver.SetProblem(L).Compute();
    math::matxxr inv = solver.Inverse();
    (math::write_npy_v10(dest + "/laplacian.npy", inv));
  }

  {
    math::spmatr P = ts->GetMassMatrix();
    math::spmatr_for_each(P, [](idx, idx, real& val) {
      val = 1.0;
    });
    P = (P * P).eval();
    P.makeCompressed();
    math::matxxi non_zero_entries(2, P.nonZeros());
    idx cnt = 0;
    math::spmatr_for_each(P, [&](idx i, idx j, real) {
      non_zero_entries.col(cnt++) = math::vec2i{i, j};
    });

    math::write_npy_v10(dest + "/mass_pattern.npy", non_zero_entries);
  }

  out = create_entity();
  add_component<gl::Mesh>(out);
  add_component<gl::Lines>(out);
  update_rendering();
  connect<gl::UiRenderEvent, &ui_callback>();
  std::cout << ts->GetOptions() << std::endl;
  AX_CHECK_OK(gl::enter_main_loop());
  ts.reset();
  clean_up();
  return 0;
}
