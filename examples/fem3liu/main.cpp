#include <imgui.h>

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
#include "ax/fem/timestepper/quasi_newton.hpp"
#include "ax/fem/topo.hpp"
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
ABSL_FLAG(int, N, 3, "Num of division.");
ABSL_FLAG(bool, flip_yz, false, "flip yz");
ABSL_FLAG(std::string, scene, "bend", "id of scene, 0 for twist, 1 for bend.");
ABSL_FLAG(std::string, elast, "stable_neohookean",
          "Hyperelasticity model, nh=Neohookean arap=Arap");
ABSL_FLAG(std::string, optim, "newton", "optimizer newton 'naive' or 'liu'");
ABSL_FLAG(std::string, device, "gpu", "cpu or gpu");
ABSL_FLAG(bool, optopo, true, "Optimize topology using RCM.");
ABSL_FLAG(std::string, lbfgs, "laplacian", "naive, laplacian, hard");
ABSL_FLAG(double, youngs, 1e7, "Youngs");

int nx;
using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::vec2r lame;

#define SCENE_TWIST 0
#define SCENE_BEND 1
#define SCENE_ARMADILLO_DRAG 2
#define SCENE_ARMADILLO_EXTREME 3
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
  auto &elast = ts->GetElasticity();
  elast.Update(ts->GetDisplacement(), fem::ElasticityUpdateLevel::kEnergy);
  elast.UpdateEnergy();
  elast.GatherEnergyToVertices();
  auto e_per_vert = elast.GetEnergyOnVertices();

  real m = 0, M = 0;
  m = e_per_vert.minCoeff();
  M = e_per_vert.maxCoeff();
  std::cout << "Energy: " << m << " " << M << std::endl;

  gl::Colormap cmap((m), (M));
  mesh.colors_.topRows<3>() = cmap(e_per_vert);
}

static bool running = false;
float dt = 1e-2;
math::vecxr fps;

void handle_armadillo_drags(fem::TriMesh<3>& mesh, real T) {
  using namespace ax::math;
  static std::vector<idx> dirichlet_handles;
  static std::vector<real> y_vals;
  if (dirichlet_handles.empty()) {
    // first time call.
    vec3r center{0, 0.2, 0};
    real radius = 0.2;
    for (idx i = 0; i < mesh.GetNumVertices(); ++i) {
      auto X = mesh.GetVertex(i);
      if (norm(X - center) < radius) {
        dirichlet_handles.push_back(i);
        y_vals.push_back(X.y());
        mesh.MarkDirichletBoundary(i, 0, X.x());
        mesh.MarkDirichletBoundary(i, 1, X.y());
        mesh.MarkDirichletBoundary(i, 2, X.z());
      }
    }
  }

  for (size_t i = 0; i < dirichlet_handles.size(); ++i) {
    idx iv = dirichlet_handles[i];
    real v = y_vals[i] + sin(10 * T) * 0.5;
    mesh.MarkDirichletBoundary(iv, 1, v);
  }
}

void handle_armadillo_extreme(fem::TriMesh<3>& mesh, real T) {
  using namespace ax::math;
  static std::vector<idx> l_dirichlet_handles;
  static std::vector<idx> r_dirichlet_handles;
  static std::vector<real> x_vals_l, x_vals_r;
  if (l_dirichlet_handles.empty()) {
    // first time call.
    vec3r center{0.7, 0.6, 0.6};
    real radius = 0.1;
    for (idx i = 0; i < mesh.GetNumVertices(); ++i) {
      auto X = mesh.GetVertex(i);
      if (norm(X - center) < radius) {
        mesh.MarkDirichletBoundary(i, 0, X.x());
        mesh.MarkDirichletBoundary(i, 1, X.y());
        mesh.MarkDirichletBoundary(i, 2, X.z());
        l_dirichlet_handles.push_back(i);
        x_vals_l.push_back(X.x());
      }
    }
  }

  if (r_dirichlet_handles.empty()) {
    // first time call.
    vec3r center{-0.7, 0.77, 0.4};
    real radius = 0.1;
    for (idx i = 0; i < mesh.GetNumVertices(); ++i) {
      auto X = mesh.GetVertex(i);
      if (norm(X - center) < radius) {
        mesh.MarkDirichletBoundary(i, 0, X.x());
        mesh.MarkDirichletBoundary(i, 1, X.y());
        mesh.MarkDirichletBoundary(i, 2, X.z());
        r_dirichlet_handles.push_back(i);
        x_vals_r.push_back(X.x());
      }
    }
  }
  for (size_t i = 0; i < l_dirichlet_handles.size(); ++i) {
    idx iv = l_dirichlet_handles[i];
    real v = x_vals_l[i] + T * 0.5;
    mesh.MarkDirichletBoundary(iv, 0, v);
  }
  for (size_t i = 0; i < r_dirichlet_handles.size(); ++i) {
    idx iv = r_dirichlet_handles[i];
    real v = x_vals_r[i] - T * 0.5;
    mesh.MarkDirichletBoundary(iv, 0, v);
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
    if (scene == SCENE_TWIST) {
      // Apply some Dirichlet BC
      math::mat3r rotate = Eigen::AngleAxis<real>(dt, math::vec3r::UnitX()).matrix();
      for (auto i : utils::iota(vert.cols())) {
        const auto& position = vert.col(i);
        if (-position.x() > 4.9) {
          // Mark as dirichlet bc.
          math::vec3r p = rotate * position;
          ts->GetMesh()->MarkDirichletBoundary(i, 0, p.x());
          ts->GetMesh()->MarkDirichletBoundary(i, 1, p.y());
          ts->GetMesh()->MarkDirichletBoundary(i, 2, p.z());
        }
      }
    } else if (scene == SCENE_ARMADILLO_DRAG) {
      handle_armadillo_drags(*ts->GetMesh(), frame * dt);
    } else if (scene == SCENE_ARMADILLO_EXTREME) {
      handle_armadillo_extreme(*ts->GetMesh(), frame * dt);
    }

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
  auto sname = absl::GetFlag(FLAGS_scene);
  if (sname == "twist") {
    scene = SCENE_TWIST;
  } else if (sname == "bend") {
    scene = SCENE_BEND;
  } else if (sname == "drag") {
    scene = SCENE_ARMADILLO_DRAG;
  } else if (sname == "extreme") {
    scene = SCENE_ARMADILLO_EXTREME;
  } else {
    AX_CHECK(false) << "Invalid scene name.";
  }

  nx = absl::GetFlag(FLAGS_N);

  std::string tet_file, vet_file;

  lame = fem::elasticity::compute_lame(absl::GetFlag(FLAGS_youngs), 0.45);
  if (scene == SCENE_TWIST || scene == SCENE_BEND) {
    tet_file = utils::get_asset("/mesh/npy/beam_high_res_elements.npy");
    vet_file = utils::get_asset("/mesh/npy/beam_high_res_vertices.npy");
    // tet_file = utils::get_asset("/mesh/npy/beam_mid_res_elements.npy");
    // vet_file = utils::get_asset("/mesh/npy/beam_mid_res_vertices.npy");
  } else if (scene == SCENE_ARMADILLO_DRAG || scene == SCENE_ARMADILLO_EXTREME) {
    tet_file = utils::get_asset("/mesh/npy/armadillo_low_res_larger_elements.npy");
    vet_file = utils::get_asset("/mesh/npy/armadillo_low_res_larger_vertices.npy");
  }

  auto tet = math::read_npy_v10_idx(tet_file);
  auto vet = math::read_npy_v10_real(vet_file);
  auto cube = geo::tet_cube(0.5, 10 * nx, nx, nx);
  cube.vertices_.row(0) *= 10;
  input_mesh.indices_ = tet.transpose();
  input_mesh.vertices_ = vet.transpose();
  // input_mesh = cube;

  if (auto opt = absl::GetFlag(FLAGS_optim); opt == "liu") {
    ts = std::make_unique<fem::Timestepper_QuasiNewton<3>>(std::make_shared<fem::TriMesh<3>>());
    auto strategy = absl::GetFlag(FLAGS_lbfgs);
    auto p_ts = reinterpret_cast<fem::Timestepper_QuasiNewton<3>*>(ts.get());
    if (strategy == "naive") {
      std::cout << "LBFGS: Naive" << std::endl;
      p_ts->SetOptions({{"lbfgs_strategy", "kNaive"}});
    } else if (strategy == "laplacian") {
      std::cout << "LBFGS: Liu17" << std::endl;
      p_ts->SetOptions({{"lbfgs_strategy", "kLaplacian"}});
    } else {
      std::cout << "LBFGS: Hard" << std::endl;
      p_ts->SetOptions({{"lbfgs_strategy", "kHard"}});
    }
  } else if (opt == "newton") {
    std::cout << "Newton" << std::endl;
    ts = std::make_unique<fem::Timestepper_NaiveOptim<3>>(std::make_shared<fem::TriMesh<3>>());
  } else {
    AX_CHECK(false) << "Invalid optimizer name, expect 'liu' or 'newton'";
  }
  ts->SetYoungs(absl::GetFlag(FLAGS_youngs));
  ts->SetPoissonRatio(0.45);

  ts->GetMesh()->SetMesh(input_mesh.indices_, input_mesh.vertices_);
  ts->GetMesh()->SetNumDofPerVertex(3);
  ts->GetMesh()->ResetAllBoundaries();

  input_mesh.vertices_ = ts->GetMesh()->GetVertices();
  input_mesh.indices_ = ts->GetMesh()->GetElements();
  if (scene == SCENE_TWIST || scene == SCENE_BEND) {
    for (auto i : utils::iota(input_mesh.vertices_.cols())) {
      const auto& position = input_mesh.vertices_.col(i);
      if (math::abs(position.x()) > 4.9) {
        // Mark as dirichlet bc.
        if (scene == SCENE_TWIST || position.x() > 4.9) {
          ts->GetMesh()->MarkDirichletBoundary(i, 0, position.x());
          ts->GetMesh()->MarkDirichletBoundary(i, 1, position.y());
          ts->GetMesh()->MarkDirichletBoundary(i, 2, position.z());
        }
      }
    }
  } else if (scene == SCENE_ARMADILLO_DRAG) {
    handle_armadillo_drags(*ts->GetMesh(), 0);
  } else if (scene == SCENE_ARMADILLO_EXTREME) {
    handle_armadillo_extreme(*ts->GetMesh(), 0);
  }

  ts->SetDensity(1e3);
  ts->SetupElasticity(absl::GetFlag(FLAGS_elast),
#ifdef AX_HAS_CUDA
                      absl::GetFlag(FLAGS_device)
#else
                      "cpu"
#endif
  );
  AX_CHECK_OK(ts->Initialize());

  std::cout << "Running Parameters: " << ts->GetOptions() << std::endl;

  if (scene == SCENE_TWIST || scene == SCENE_ARMADILLO_DRAG || scene == SCENE_ARMADILLO_EXTREME) {
    ts->SetExternalAcceleration(math::field3r::Zero(3, ts->GetMesh()->GetNumVertices()));
  } else {
    ts->SetExternalAccelerationUniform(math::vec3r{0, -9.8, 0});
  }

  ts->BeginSimulation(dt);
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
