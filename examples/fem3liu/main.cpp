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
ABSL_FLAG(std::string, scene, "twist", "id of scene, 0 for twist, 1 for bend.");
ABSL_FLAG(std::string, elast, "nh", "Hyperelasticity model, nh=Neohookean arap=Arap");
ABSL_FLAG(std::string, optim, "liu", "optimizer newton 'naive' or 'liu'");
ABSL_FLAG(std::string, device, "gpu", "cpu or gpu");
ABSL_FLAG(bool, optopo, true, "Optimize topology using RCM.");
ABSL_FLAG(std::string, lbfgs, "naive", "naive, laplacian, hard");

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
  mesh.vertices_ = ts->GetMesh().GetVertices();
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
  ts->GetElasticity().Update(ts->GetMesh().GetVertices(),
                                                fem::ElasticityUpdateLevel::kEnergy);
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
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh().GetNumElements(),
              ts->GetMesh().GetNumVertices());
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
    } else if (scene == SCENE_ARMADILLO_DRAG) {
      handle_armadillo_drags(ts->GetMesh(), frame * dt);
    } else if (scene == SCENE_ARMADILLO_EXTREME) {
      handle_armadillo_extreme(ts->GetMesh(), frame * dt);
    }

    auto time_start = ax::utils::GetCurrentTimeNanos();
    AX_CHECK_OK(ts->Step(dt));
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

  if (scene == SCENE_TWIST || scene == SCENE_BEND) {
    lame = fem::elasticity::compute_lame(1e7, 0.45);
    tet_file = utils::get_asset("/mesh/npy/beam_high_res_elements.npy");
    vet_file = utils::get_asset("/mesh/npy/beam_high_res_vertices.npy");
  } else if (scene == SCENE_ARMADILLO_DRAG || scene == SCENE_ARMADILLO_EXTREME) {
    lame = fem::elasticity::compute_lame(1e6, 0.45);
    tet_file = utils::get_asset("/mesh/npy/armadillo_low_res_larger_elements.npy");
    vet_file = utils::get_asset("/mesh/npy/armadillo_low_res_larger_vertices.npy");
  }

  auto cube = geo::tet_cube(10, 2, 2, 2);
  auto tet = math::read_npy_v10_idx(tet_file);
  auto vet = math::read_npy_v10_real(vet_file);
  AX_CHECK(tet.ok() && vet.ok()) << "Failed to load mesh.";
  input_mesh.indices_ = tet->transpose();
  input_mesh.vertices_ = vet->transpose();
  // fix_negative_volume(input_mesh.indices_, input_mesh.vertices_);

  if (auto opt = absl::GetFlag(FLAGS_optim); opt == "liu") {
    ts = std::make_unique<fem::Timestepper_QuasiNewton<3>>(std::make_shared<fem::TriMesh<3>>());
    auto strategy = absl::GetFlag(FLAGS_lbfgs);
  } else if (opt == "newton") {
    ts = std::make_unique<fem::Timestepper_NaiveOptim<3>>(std::make_shared<fem::TriMesh<3>>());
  } else {
    AX_CHECK(false) << "Invalid optimizer name, expect 'liu' or 'newton'";
  }
  ts->SetLame(lame);

  AX_CHECK_OK(ts->GetMesh().SetMesh(input_mesh.indices_, input_mesh.vertices_));
  if (auto opt = absl::GetFlag(FLAGS_optopo); opt) {
    auto [p, ip] = fem::optimize_topology<3>(input_mesh.indices_, input_mesh.vertices_.cols());
    ts->GetMesh().ApplyPermutation(p, ip);
  }

  input_mesh.vertices_ = ts->GetMesh().GetVertices();
  input_mesh.indices_ = ts->GetMesh().GetElements();
  if (scene == SCENE_TWIST || scene == SCENE_BEND) {
    for (auto i : utils::iota(input_mesh.vertices_.cols())) {
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
  } else if (scene == SCENE_ARMADILLO_DRAG) {
    handle_armadillo_drags(ts->GetMesh(), 0);
  } else if (scene == SCENE_ARMADILLO_EXTREME) {
    handle_armadillo_extreme(ts->GetMesh(), 0);
  }

  ts->SetDensity(1e3);
  AX_CHECK_OK(ts->Init());

#ifdef AX_HAS_CUDA
  if (auto device = absl::GetFlag(FLAGS_device); device == "cpu") {
    ts->SetupElasticity<fem::elasticity::StableNeoHookean, fem::ElasticityCompute_CPU>();
  } else if (device == "gpu") {
    ts->SetupElasticity<fem::elasticity::StableNeoHookean, fem::ElasticityCompute_GPU>();
  }
#else
  ts->SetupElasticity<fem::elasticity::StableNeoHookean, fem::ElasticityCompute_CPU>();
#endif

  if (scene == SCENE_TWIST) {
    ts->SetExternalAcceleration(math::field3r::Zero(3, ts->GetMesh().GetNumVertices()));
  }
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
