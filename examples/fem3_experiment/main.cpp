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
#include "ax/math/io.hpp"
#include "ax/utils/iota.hpp"
#include "ax/utils/asset.hpp"

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

math::spmatr laplacian;
std::unique_ptr<math::SparseSolverBase> laplacian_solver, hyper_solver;

int scene;

UPtr<fem::TimeStepperBase<3>> ts;

real KL_div(math::matxxr sigma_P, math::matxxr sigma_Q) {
  auto [eigvec_P, eigval_P] = math::eig(sigma_P);
  auto [eigvec_Q, eigval_Q] = math::eig(sigma_Q);
  real logdet_P_over_Q = 0;
  for (auto i : utils::iota(eigval_P.size())) {
    logdet_P_over_Q += log(eigval_P(i) / (eigval_Q(i) + math::epsilon<>));
  }

  math::matxxr inv_P = sigma_P.inverse();
  real KL = 0.5 * (inv_P * sigma_Q).trace() - 0.5 * 3 + 0.5 * logdet_P_over_Q;
  return KL;
}

real KL_symmetric(math::matxxr sigma_P, math::matxxr sigma_Q) {
  math::matxxr inv_P = sigma_P.inverse(), inv_Q = sigma_Q.inverse();
  real KL = 0.5 * (inv_P * sigma_Q).trace() + 0.5 * (inv_Q * sigma_P).trace() - 3;
  return KL;
}

real cond(math::matxxr A) {
  auto [eig, eigval] = math::eig(A);
  return eigval.maxCoeff() / eigval.minCoeff();
}

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

  ts->GetElasticity().Update(ts->GetDisplacement(), fem::ElasticityUpdateLevel::kEnergy);
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
math::vecxr cs_dist_eigen, eval, l2_dist_eigen, relative_l2_dist_eigen;
math::vecxr cs_dist_invs, l2_dist_invs, relative_l2_dist_invs;
math::vecxr cs_dist_units, l2_dist_units, relative_l2_dist_units;
math::vecxr cs_dist_randoms, l2_dist_randoms, relative_l2_dist_randoms;
math::field3r u0, u1;

void update_eigen_evaluation() {
  idx const dofs = ts->GetMesh()->GetNumVertices() * 3;
  math::vecxr dx = (u1 - u0).reshaped();
  ts->GetMesh()->FilterVector(dx, true);
  // stiffness:
  // find the eigen values of K
  math::spmatr A = ts->Hessian(u1);
  hyper_solver->SetProblem(A).Compute();

  auto [vec, val] = math::eig(A.toDense());
  // Eigen::GeneralizedSelfAdjointEigenSolver<math::matxxr> es(K, M);
  // auto vec = es.eigenvectors();
  // auto val = es.eigenvalues();
  // Decompose dx into the eigenvectors of K

  static int distance_measurement = 0;  // 0 => l2, 1 => cosine_similarity

  auto cosine_similarity
      = [](const math::vecxr& a, const math::vecxr& b) { return a.dot(b) / (a.norm() * b.norm()); };

  auto l2 = [](const math::vecxr& a, const math::vecxr& b) { return math::norm(a - b) / a.size(); };

  auto relative_l2
      = [l2](const math::vecxr& a, const math::vecxr& b) { return l2(a, b) / math::norm(b); };

  cs_dist_eigen.resize(val.size());
  l2_dist_eigen.resize(val.size());
  relative_l2_dist_eigen.resize(val.size());
  for (auto i : utils::iota(val.size())) {
    math::vecxr laplacian_applied = laplacian * vec.col(i);
    math::vecxr stiffness_applied = A * vec.col(i);
    l2_dist_eigen(i) = l2(laplacian_applied, stiffness_applied);
    cs_dist_eigen(i) = cosine_similarity(laplacian_applied, stiffness_applied);
    relative_l2_dist_eigen(i) = l2(laplacian_applied, stiffness_applied) / val(i);
  }

  relative_l2_dist_eigen /= relative_l2_dist_eigen.maxCoeff();

  cs_dist_invs.resize(val.size());
  l2_dist_invs.resize(val.size());
  relative_l2_dist_invs.resize(val.size());
  idx const nDof = ts->GetMesh()->GetNumVertices() * 3;
  math::matxxr A_inverse = A.toDense().inverse();
  std::tie(vec, val) = math::eig(A_inverse);
  for (auto i : utils::iota(val.size())) {
    math::vecxr evec = math::normalized(vec.col(i));
    auto laplacian_applied = laplacian_solver->Solve(evec, math::vecxr::Zero(nDof)).solution_;
    auto stiffness_applied = hyper_solver->Solve(evec, math::vecxr::Zero(nDof)).solution_;

    l2_dist_invs(i) = l2(laplacian_applied, stiffness_applied);
    cs_dist_invs(i) = cosine_similarity(laplacian_applied, stiffness_applied);
    relative_l2_dist_invs(i) = relative_l2(laplacian_applied, stiffness_applied);
  }

  cs_dist_units.resize(val.size());
  l2_dist_units.resize(val.size());
  relative_l2_dist_units.resize(val.size());
  for (auto i : utils::iota(val.size())) {
    math::vecxr unit_i = math::vecxr::Zero(val.size());
    unit_i(i) = 1;
    auto laplacian_applied = laplacian_solver->Solve(unit_i, math::vecxr::Zero(nDof)).solution_;
    auto stiffness_applied = hyper_solver->Solve(unit_i, math::vecxr::Zero(nDof)).solution_;

    l2_dist_units(i) = l2(laplacian_applied, stiffness_applied);
    cs_dist_units(i) = cosine_similarity(laplacian_applied, stiffness_applied);
    relative_l2_dist_units(i) = relative_l2(laplacian_applied, stiffness_applied);
  }

  cs_dist_randoms.resize(val.size());
  l2_dist_randoms.resize(val.size());
  relative_l2_dist_randoms.resize(val.size());
  for (auto i : utils::iota(val.size())) {
    math::vecxr unit_i = math::vecxr::Zero(val.size());
    unit_i.setRandom().normalize();
    auto laplacian_applied = laplacian_solver->Solve(unit_i, math::vecxr::Zero(nDof)).solution_;
    auto stiffness_applied = hyper_solver->Solve(unit_i, math::vecxr::Zero(nDof)).solution_;

    l2_dist_randoms(i) = l2(laplacian_applied, stiffness_applied);
    cs_dist_randoms(i) = cosine_similarity(laplacian_applied, stiffness_applied);
    relative_l2_dist_randoms(i) = relative_l2(laplacian_applied, stiffness_applied);
  }
  eval = val * dt * dt;

  std::cout << "Average cs for eigen: " << cs_dist_eigen.mean() << std::endl;
  std::cout << "Average l2 for eigen: " << l2_dist_eigen.mean() << std::endl;
  std::cout << "Average rl for eigen: " << relative_l2_dist_eigen.mean() << std::endl;
  std::cout << "Average cs for inv: " << cs_dist_invs.mean() << std::endl;
  std::cout << "Average l2 for inv: " << l2_dist_invs.mean() << std::endl;
  std::cout << "Average rl for inv: " << relative_l2_dist_invs.mean() << std::endl;
  std::cout << "Average cs for unit: " << cs_dist_units.mean() << std::endl;
  std::cout << "Average l2 for unit: " << l2_dist_units.mean() << std::endl;
  std::cout << "Average rl for unit: " << relative_l2_dist_units.mean() << std::endl;
  std::cout << "Average cs for random: " << cs_dist_randoms.mean() << std::endl;
  std::cout << "Average l2 for random: " << l2_dist_randoms.mean() << std::endl;
  std::cout << "Average rl for random: " << relative_l2_dist_randoms.mean() << std::endl;
}

void update_KL_div() {
  auto u = ts->GetDisplacement();
  math::matxxr K = ts->Hessian(u, true).toDense();
  math::matxxr L = laplacian.toDense();
  math::matxxr I = math::matxxr::Identity(K.rows(), K.cols());

  std::cout << "KL(K, L)" << KL_div(K, L) << std::endl;
  std::cout << "KL(L, K)" << KL_div(L, K) << std::endl;
  std::cout << "KL(K, I)" << KL_div(K, I) << std::endl;
  std::cout << "KL(I, K)" << KL_div(I, K) << std::endl;

  std::cout << "KL_symmetric(K, L)" << KL_symmetric(K, L) << std::endl;
  std::cout << "KL_symmetric(L, K)" << KL_symmetric(L, K) << std::endl;
  std::cout << "KL_symmetric(K, I)" << KL_symmetric(K, I) << std::endl;
  std::cout << "KL_symmetric(I, K)" << KL_symmetric(I, K) << std::endl;

  std::cout << "cond(L^-1 K)" << cond(L.inverse() * K) << std::endl;
  std::cout << "cond(K)" << cond(K) << std::endl;
}

void ui_callback(gl::UiRenderEvent) {
  ImGui::Begin("FEM");
  ImGui::Checkbox("Running", &running);
  ImGui::InputFloat("dt", &dt);
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh()->GetNumElements(),
              ts->GetMesh()->GetNumVertices());

  static bool enab_KL = false;
  if (ImGui::Button("Step") || running) {
    ts->BeginTimestep(dt);
    u0 = ts->GetPosition();
    ts->SolveTimestep();
    u1 = ts->GetSolution();
    ts->EndTimestep();

    auto const& trajectory = ts->GetLastTrajectory();

    update_rendering();

    if (scene == SCENE_TWIST) {
      // Apply some Dirichlet BC
      math::mat3r rotate = Eigen::AngleAxis<real>(dt, math::vec3r::UnitX()).matrix();
      u0 = ts->GetPosition();
      auto m = ts->GetMesh();
      for (auto i : utils::iota(u0.cols())) {
        const auto& position = u0.col(i);
        if (-position.x() > 1.9) {
          // Mark as dirichlet bc.
          math::vec3r p = rotate * position;
          m->MarkDirichletBoundary(i, 0, p.x());
          m->MarkDirichletBoundary(i, 1, p.y());
          m->MarkDirichletBoundary(i, 2, p.z());
        }
      }
    }

    if (ImGui::Checkbox("Enable KL", &enab_KL); enab_KL) {
      update_KL_div();
    }
  }

  if (ImPlot::BeginPlot("eig")) {
    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
    // ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::PlotLine("cs", cs_dist_eigen.data(), cs_dist_eigen.size());
    ImPlot::PlotLine("eigen", eval.data(), eval.size());
    ImPlot::PlotLine("l2", l2_dist_eigen.data(), l2_dist_eigen.size());
    ImPlot::PlotLine("relative_l2", relative_l2_dist_eigen.data(), relative_l2_dist_eigen.size());
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("inv")) {
    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
    // ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::PlotLine("cs", cs_dist_invs.data(), cs_dist_invs.size());
    ImPlot::PlotLine("l2", l2_dist_invs.data(), l2_dist_invs.size());
    ImPlot::PlotLine("relative_l2", relative_l2_dist_invs.data(), relative_l2_dist_invs.size());
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("unit")) {
    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
    // ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::PlotLine("cs", cs_dist_units.data(), cs_dist_units.size());
    ImPlot::PlotLine("l2", l2_dist_units.data(), l2_dist_units.size());
    ImPlot::PlotLine("relative_l2", relative_l2_dist_units.data(), relative_l2_dist_units.size());
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("random")) {
    ImPlot::SetupAxis(ImAxis_X1, nullptr, ImPlotAxisFlags_AutoFit);
    // ImPlot::SetupAxis(ImAxis_Y1, nullptr, ImPlotAxisFlags_AutoFit);
    ImPlot::PlotLine("cs", cs_dist_randoms.data(), cs_dist_randoms.size());
    ImPlot::PlotLine("l2", l2_dist_randoms.data(), l2_dist_randoms.size());
    ImPlot::PlotLine("relative_l2", relative_l2_dist_randoms.data(),
                     relative_l2_dist_randoms.size());
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

  std::string tet_file, vet_file, resolution = "low";
  tet_file = utils::get_asset("/mesh/npy/beam_" + resolution + "_res_elements.npy");
  vet_file = utils::get_asset("/mesh/npy/beam_" + resolution + "_res_vertices.npy");

  auto tet = math::read_npy_v10_idx(tet_file);
  auto vet = math::read_npy_v10_real(vet_file);
  input_mesh.indices_ = tet.transpose();
  input_mesh.vertices_ = vet.transpose();

  ts = std::make_unique<fem::Timestepper_QuasiNewton<3>>(std::make_unique<fem::TriMesh<3>>());
  ts->SetYoungs(youngs);
  ts->SetPoissonRatio(poisson);

  ts->GetMesh()->SetMesh(input_mesh.indices_, input_mesh.vertices_);
  auto m = ts->GetMesh();
  for (auto i : utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (position.x() > 4.9) {
      m->MarkDirichletBoundary(i, 0, position.x());
      m->MarkDirichletBoundary(i, 1, position.y());
      m->MarkDirichletBoundary(i, 2, position.z());
    }
  }

  ts->SetupElasticity("stable_neohookean", "cpu");
  ts->SetDensity(1e3);

  ts->SetOptions({
      {"tol_var", 0.0},
      {"record_trajectory", 1},
      // {"lbfgs_strategy", "kReservedForExperimental"},
      {"lbfgs_strategy", "kLaplacian"},
      {"rel_tol_grad", 1e-2},
      // {"lbfgs_opt", {{"check_approx_quality", true}}}
  });

  AX_CHECK_OK(ts->Initialize());
  ts->BeginSimulation(dt);
  std::cout << "==> Initialized" << std::endl;
  std::cout << "Timestepper Options: " << ts->GetOptions() << std::endl;
  ts->SetExternalAccelerationUniform(math::vec3r(0, -9.8, 0));

  {
    real lambda = lame[0] + 5.0 / 6.0 * lame[1], mu = 4.0 / 3.0 * lame[1];
    real W = 2 * mu + lambda;
    auto L = fem::LaplaceMatrixCompute<3>{*ts->GetMesh()}(W);
    laplacian = math::kronecker_identity<3>(L);
    auto const& M = ts->GetMassMatrix();
    laplacian = M + dt * dt * laplacian;
    ts->GetMesh()->FilterMatrixFull(laplacian);
    laplacian.makeCompressed();
  }

  laplacian_solver = math::SparseSolverBase::Create(ax::math::SparseSolverKind::kLLT);
  laplacian_solver->SetProblem(laplacian).Compute();
  hyper_solver = math::SparseSolverBase::Create(ax::math::SparseSolverKind::kLLT);

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
