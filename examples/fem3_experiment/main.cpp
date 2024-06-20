#include <imgui.h>
#include <implot.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/elasticity/base.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/fem/timestepper.hpp"
#include "ax/fem/timestepper/quasi_newton.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/iota.hpp"

ABSL_FLAG(std::string, input, "plane.obj", "Input 2D Mesh.");
ABSL_FLAG(int, N, 2, "Num of division.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");
ABSL_FLAG(int, scene, 0, "id of scene, 0 for twist, 1 for bend.");
int nx;
using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;

#define SCENE_TWIST 0
#define SCENE_BEND 1

math::sp_matxxr laplacian;
std::unique_ptr<math::SparseSolverBase> laplacian_solver, hyper_solver;

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
math::vecxr cs_dist_eigen, eval, l2_dist_eigen, relative_l2_dist_eigen;
math::vecxr cs_dist_units, l2_dist_units, relative_l2_dist_units;

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
    math::field3r u0 = ts->GetPosition();
    ts->SolveTimestep();
    math::field3r u1 = ts->GetSolution();
    ts->EndTimestep();

    auto const& trajectory = ts->GetLastTrajectory();

    idx const dofs = ts->GetMesh().GetNumVertices() * 3;
    math::vecxr dx = (u1 - u0).reshaped();
    ts->GetMesh().FilterVector(dx, true);
    // stiffness:
    // find the eigen values of K
    math::sp_matxxr A = ts->Hessian(u1);
    math::LinsysProblem_Sparse pro;
    pro.A_ = A;
    hyper_solver->Analyse(pro);

    auto [vec, val] = math::eig(A.toDense());
    // Eigen::GeneralizedSelfAdjointEigenSolver<math::matxxr> es(K, M);
    // auto vec = es.eigenvectors();
    // auto val = es.eigenvalues();
    // Decompose dx into the eigenvectors of K

    static int distance_measurement = 0;  // 0 => l2, 1 => cosine_similarity

    auto cosine_similarity = [](const math::vecxr& a, const math::vecxr& b) {
      return a.dot(b) / (a.norm() * b.norm());
    };

    auto l2
        = [](const math::vecxr& a, const math::vecxr& b) { return math::norm(a - b) / a.size(); };

    auto relative_l2 = [](const math::vecxr& a, const math::vecxr& b) {
      return math::norm(a - b) / math::norm(b);
    };

    cs_dist_eigen.resize(val.size());
    l2_dist_eigen.resize(val.size());
    relative_l2_dist_eigen.resize(val.size());
    for (auto i : utils::iota(val.size())) {
      math::vecxr laplacian_applied = laplacian * math::normalized(vec.col(i));
      math::vecxr stiffness_applied = A * math::normalized(vec.col(i));
      l2_dist_eigen(i) = l2(laplacian_applied, stiffness_applied);
      cs_dist_eigen(i) = cosine_similarity(laplacian_applied, stiffness_applied);
      relative_l2_dist_eigen(i) = relative_l2(laplacian_applied, stiffness_applied);
    }

    cs_dist_units.resize(val.size());
    l2_dist_units.resize(val.size());
    relative_l2_dist_units.resize(val.size());
    // idx const nDof = ts->GetMesh().GetNumVertices() * 3;
    // for (auto i : utils::iota(val.size())) {
    //   math::vecxr evec = vec.col(i);
    //   auto laplacian_applied = laplacian_solver->Solve(evec, math::vecxr::Zero(nDof)).solution_;
    //   auto stiffness_applied = hyper_solver->Solve(evec, math::vecxr::Zero(nDof)).solution_;
    //
    //   l2_dist_units(i) = l2(laplacian_applied, stiffness_applied);
    //   cs_dist_units(i) = cosine_similarity(laplacian_applied, stiffness_applied);
    //   relative_l2_dist_units(i) = relative_l2(laplacian_applied, stiffness_applied);
    // }

    // check the relationship between M_dt2_L and M + dt2 * K

    eval = val * dt * dt;
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
    std::cout << "Average cs for eigen: " << cs_dist_eigen.mean() << std::endl;
    std::cout << "Average l2 for eigen: " << l2_dist_eigen.mean() << std::endl;
    std::cout << "Average rl for eigen: " << relative_l2_dist_eigen.mean() << std::endl;
    std::cout << "Average cs for unit: " << cs_dist_units.mean() << std::endl;
    std::cout << "Average l2 for unit: " << l2_dist_units.mean() << std::endl;
    std::cout << "Average rl for unit: " << relative_l2_dist_units.mean() << std::endl;
  }

  if (ImPlot::BeginPlot("eig")) {
    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::PlotLine("cs", cs_dist_eigen.data(), cs_dist_eigen.size());
    ImPlot::PlotLine("eigen", eval.data(), eval.size());
    ImPlot::PlotLine("l2", l2_dist_eigen.data(), l2_dist_eigen.size());
    ImPlot::PlotLine("relative_l2", relative_l2_dist_eigen.data(), relative_l2_dist_eigen.size());
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("unit")) {
    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::PlotLine("cs", cs_dist_units.data(), cs_dist_units.size());
    ImPlot::PlotLine("l2", l2_dist_units.data(), l2_dist_units.size());
    ImPlot::PlotLine("relative_l2", relative_l2_dist_units.data(), relative_l2_dist_units.size());
    ImPlot::EndPlot();
  }

  ImGui::End();
}

int main(int argc, char** argv) {
  ax::gl::init(argc, argv);
  fps.setZero(100);
  scene = absl::GetFlag(FLAGS_scene);

  real const poisson = 0.45, youngs = 1e7;

  lame = fem::elasticity::compute_lame(poisson, youngs);
  nx = absl::GetFlag(FLAGS_N);

  auto cube = geo::tet_cube(0.5, nx * 4, nx, nx);
  input_mesh.vertices_ = std::move(cube.vertices_);
  input_mesh.indices_ = std::move(cube.indices_);
  input_mesh.vertices_.row(0) *= 4;

  ts = std::make_unique<fem::Timestepper_QuasiNewton<3>>(std::make_unique<fem::TriMesh<3>>());
  ts->SetYoungs(youngs);
  ts->SetPoissonRatio(poisson);

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

  ts->SetupElasticity("stable_neohookean", "cpu");
  ts->SetDensity(1e3);

  ts->SetOptions({
      {"tol_var", 0.0},
      {"record_trajectory", 1},
      {"lbfgs_strategy", "kReservedForExperimental"},
      {"rel_tol_grad", 1e-2},
      {"lbfgs_opt", {{"check_approx_quality", true}}}
  });

  AX_CHECK_OK(ts->Initialize());
  ts->BeginSimulation(dt);
  std::cout << "==> Initialized" << std::endl;
  std::cout << "Timestepper Options: " << ts->GetOptions() << std::endl;
  ts->SetExternalAccelerationUniform(math::vec3r(0, -9.8, 0));

  {
    real lambda = lame[0] + 5.0 / 6.0 * lame[1], mu = 4.0 / 3.0 * lame[1];
    real W = 2 * mu + lambda;
    auto L = fem::LaplaceMatrixCompute<3>{ts->GetMesh()}(W);
    laplacian = math::kronecker_identity<3>(L);
    auto const& M = ts->GetMassMatrix();
    laplacian = M + dt * dt * laplacian;
    ts->GetMeshPtr()->FilterMatrixFull(laplacian);
    laplacian.makeCompressed();
  }

  laplacian_solver = math::SparseSolverBase::Create(ax::math::SparseSolverKind::kConjugateGradient);
  math::LinsysProblem_Sparse pro;
  pro.A_ = laplacian;
  laplacian_solver->Analyse(pro);

  hyper_solver = math::SparseSolverBase::Create(ax::math::SparseSolverKind::kConjugateGradient);

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
