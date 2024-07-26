#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/fem/elasticity/arap.hpp"
#include "ax/fem/elasticity/linear.hpp"
#include "ax/fem/elasticity/neohookean_bw.hpp"
#include "ax/fem/elasticity/stable_neohookean.hpp"
#include "ax/fem/elasticity/stvk.hpp"
#include "ax/fem/laplace_matrix.hpp"
#include "ax/fem/mass_matrix.hpp"
#include "ax/fem/timestepper/rom.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/fem/elasticity_gpu.cuh"
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
ABSL_FLAG(int, N, 5, "Num of division.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");
ABSL_FLAG(bool, scene, 0, "id of scene, 0 for twist, 1 for bend.");
ABSL_FLAG(int, modes, 30, "");
int nx;
using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;

#define SCENE_TWIST 0
#define SCENE_BEND 1
int scene;

std::unique_ptr<fem::TimeStepperBase<3>> ts;


void update_rendering() {
  auto &mesh = get_component<gl::Mesh>(out);
  if (mesh.indices_ .size() == 0) {
    mesh.indices_ = geo::get_boundary_triangles(input_mesh.vertices_, input_mesh.indices_);
  }
  mesh.vertices_ = ts->GetMesh()->GetVertices();
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

  ts->GetElasticity().Update(ts->GetMesh()->GetVertices(), 
      fem::ElasticityUpdateLevel::kEnergy);
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
void ui_callback(gl::UiRenderEvent ) {
  ImGui::Begin("FEM", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Checkbox("Running", &running);
  ImGui::InputFloat("dt", &dt);
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh()->GetNumElements(), ts->GetMesh()->GetNumVertices());
  if (ImGui::Button("Step") || running) {
    const auto& vert = ts->GetMesh()->GetVertices();
    // if (scene == SCENE_TWIST) {
    //   // Apply some Dirichlet BC
    //   math::mat3r rotate = Eigen::AngleAxis<real>(dt, math::vec3r::UnitX()).matrix();
    //   for (auto i : utils::iota(vert.cols())) {
    //     const auto& position = vert.col(i);
    //     if (-position.x() > 1.9) {
    //       // Mark as dirichlet bc.
    //       math::vec3r p = rotate * position;
    //       ts->GetMesh()->MarkDirichletBoundary(i, 0, p.x());
    //       ts->GetMesh()->MarkDirichletBoundary(i, 1, p.y());
    //       ts->GetMesh()->MarkDirichletBoundary(i, 2, p.z());
    //     }
    //   }
    // }

    auto time_start = ax::utils::GetCurrentTimeNanos();
    static idx frame = 0;
    auto p = (fem::TimeStepper_ROM<3>*) ts.get();
    auto K = ts->GetStiffnessMatrix(ts->GetMesh()->GetVertices(), true);
    ts->GetMesh()->FilterMatrixFull(K);
    auto [vec, val] = math::eig(K.toDense());
    p->SetBasis(vec.leftCols(100));
    // AX_CHECK_OK(ts->Step(dt));
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

  auto cube = geo::tet_cube(0.5, nx * 4, nx, nx);
  input_mesh.vertices_ = std::move(cube.vertices_);
  input_mesh.indices_ = std::move(cube.indices_);
  input_mesh.vertices_.row(0) *= 4;

  ts = std::make_unique<fem::TimeStepper_ROM<3>>(std::make_unique<fem::TriMesh<3>>());
  ts->GetMesh()->SetMesh(input_mesh.indices_, input_mesh.vertices_);
  auto p = (fem::TimeStepper_ROM<3>*) ts.get();
  auto m = ts->GetMesh();
  // p->SetBasis(math::matxxr::Identity(input_mesh.vertices_.cols() * 3, input_mesh.vertices_.cols() * 3));
  for (auto i: utils::iota(input_mesh.vertices_.cols())) {
    const auto& position = input_mesh.vertices_.col(i);
    if (position.x() > 1.9 && position.y() >= 0.499) {
      // Mark as dirichlet bc.
      m->MarkDirichletBoundary(i, 0, position.x());
      m->MarkDirichletBoundary(i, 1, position.y());
      m->MarkDirichletBoundary(i, 2, position.z());
    }
  }

  // compute M lambda v + K v = 0's solution.
  math::spmatr M_sp = fem::MassMatrixCompute<3>(*ts->GetMesh())(1);
  ts->GetMesh()->FilterMatrixDof(0, M_sp);
  math::matxxr M = M_sp.toDense();
  math::spmatr K_sp = fem::LaplaceMatrixCompute<3>(*ts->GetMesh())(1.0);
  // fem::elasticity::Linear<3> linear(lame.x(), lame.y());
  // fem::ElasticityCompute_CPU<3, fem::elasticity::Linear> e(ts->GetMeshPtr());
  // e.RecomputeRestPose();
  // e.Update(ts->GetMesh()->GetVertices(), fem::ElasticityUpdateLevel::kHessian);
  // math::spmatr K_sp = e.ComputeHessianAndGather(lame);
  ts->GetMesh()->FilterMatrixDof(0, K_sp);
  math::matxxr K = K_sp.toDense();
  Eigen::GeneralizedSelfAdjointEigenSolver<math::matxxr> solver(K+M, M);
  // Eigen::SelfAdjointEigenSolver<math::matxxr> solver(K);
  math::matxxr eigen_vectors = solver.eigenvectors().leftCols(absl::GetFlag(FLAGS_modes));
  p->SetBasis(eigen_vectors);

  AX_CHECK_OK(ts->Initialize());
  ts->SetupElasticity("stable_neohookean", "gpu");
  ts->SetDensity(1e3);
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


// #include "ax/core/init.hpp"
// #include "ax/fem/elasticity.hpp"
// #include "ax/fem/laplace_matrix.hpp"
// #include "ax/fem/mass_matrix.hpp"
// #include "ax/geometry/primitives.hpp"
// #include "ax/math/io.hpp"
// #include "ax/utils/asset.hpp"
// using namespace ax;

// std::shared_ptr<fem::TriMesh<3>> mesh;
// std::unique_ptr<fem::ElasticityComputeBase<3>> elast;  

// ABSL_FLAG(std::string, mesh, "tet_cube", "mesh name");

// int main(int argc, char** argv) {
//   init(argc, argv);
//   mesh.reset(new fem::TriMesh<3>());
//   geo::TetraMesh m;
//   if (auto name = absl::GetFlag(FLAGS_mesh); name == "tet_cube"){
//     m = geo::tet_cube(1, 2, 2, 2);
//   } else {
//     auto vert = math::read_npy_v10_real(utils::get_asset("/mesh/npy/" + name + "_vertices.npy"));
//     auto indi = math::read_npy_v10_idx(utils::get_asset("/mesh/npy/" + name + "_elements.npy"));
//     m = geo::TetraMesh(vert.value().transpose(), indi.value().transpose());
//   }
//   mesh->SetMesh(m.indices_, m.vertices_);
//   auto L = math::kronecker_identity<3>(fem::LaplaceMatrixCompute<3>(*mesh)()).toDense().eval();
//   auto M = math::kronecker_identity<3>(fem::MassMatrixCompute<3>(*mesh)(1)).toDense().eval();

//   // compute the General Eigen value problem for M and L.
//   Eigen::GeneralizedSelfAdjointEigenSolver solver(L, M);
//   // compute the first 10 eigen values.
//   math::vecxr eigen_values = solver.eigenvalues().head(10);
//   AX_CHECK_OK(math::write_npy_v10("eigen_values.npy", eigen_values));
//   // compute the first 10 eigen vectors.
//   math::matxxr eigen_vectors = solver.eigenvectors().leftCols(10);
//   std::cout <<  eigen_vectors<< std::endl;
//   std::cout <<  eigen_values.transpose() << std::endl;

//   // compute the laplace's eigen values.
//   Eigen::SelfAdjointEigenSolver<math::matxxr> adj(L);
//   std::cout << "Laplace's eigen values: " << adj.eigenvalues().transpose() << std::endl;
//   std::cout << "Laplace's eigen vectors: " << adj.eigenvectors() << std::endl;

//   math::vecxr X0 = mesh->GetVertices().reshaped();
//   math::vecxr M_X0 = M * X0;
//   math::vecxr L_X0 = L * X0;
//   // real portion = M_X0[0] / L_X0[0];
//   std::cout << "M_X0[0] / L_X0[0] = " << M_X0[0] / L_X0[0] << std::endl;
//   std::cout << M_X0.transpose() << std::endl;
//   std::cout << L_X0.transpose() << std::endl;
//   clean_up();
//   return EXIT_SUCCESS;
// }
