#include <imgui.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/fem/elasticity.hpp"
#include "ax/fem/timestepper.hpp"
#include "ax/fem/timestepper/naive_optim.hpp"
#include "ax/fem/timestepper/quasi_newton.hpp"
#include "ax/fem/trimesh.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/colormap.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/mesh.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/accessor.hpp"
#include "ax/math/io.hpp"
#include "ax/math/views.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/ndrange.hpp"
#include "ax/utils/time.hpp"
#include <range/v3/view/enumerate.hpp>
#include <range/v3/view/drop_last.hpp>

int nx;
using namespace ax;
Entity out;
geo::TetraMesh input_mesh;
math::RealVector2 lame;

#define SCENE_TWIST 0
#define SCENE_BEND 1
#define SCENE_ARMADILLO_DRAG 2
#define SCENE_ARMADILLO_EXTREME 3
int scene;

math::RealMatrixX g_basis;
std::unique_ptr<fem::TimeStepperBase<3>> ts;

void update_rendering() {
  auto& elast = ts->GetElasticity();
  elast.Update(ts->GetDisplacement(), fem::ElasticityUpdateLevel::Energy);
  elast.UpdateEnergy();
  elast.GatherEnergyToVertices();
  auto e_per_vert = elast.GetEnergyOnVertices();

  Real m = 0, M = 0;
  m = e_per_vert.minCoeff();
  M = e_per_vert.maxCoeff();
  std::cout << "Energy: " << m << " " << M << std::endl;

  gl::Colormap cmap(m, M);

  auto const& mesh = patch_component<gl::Mesh>(out, [&](gl::Mesh& mesh) {
    if (mesh.indices_.size() == 0) {
      mesh.indices_ = geo::get_boundary_triangles(input_mesh.vertices_, input_mesh.indices_);
    }
    mesh.vertices_ = ts->GetPosition();
    mesh.colors_.setOnes(4, mesh.vertices_.cols());
    mesh.use_lighting_ = false;
    mesh.colors_.topRows<3>() = cmap(e_per_vert);
  });

  patch_component<gl::Lines>(out, [&](gl::Lines& lines) {
    if (lines.indices_.size() == 0) {
      lines = gl::Lines::Create(mesh);
    }
    lines.vertices_ = mesh.vertices_;
    lines.colors_.topRows<3>().setZero();
  });
}

static bool running = false;
float dt = 1e-2;
math::RealVectorX fps;

void handle_armadillo_drags(fem::LinearMesh<3>& mesh, Real T) {
  using namespace ax::math;
  static std::vector<Index> dirichlet_handles;
  static std::vector<Real> y_vals;
  if (dirichlet_handles.empty()) {
    // first time call.
    RealVector3 center{0, 0.2, 0};
    Real radius = 0.2;
    for (Index i = 0; i < mesh.GetNumVertices(); ++i) {
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
    Index iv = dirichlet_handles[i];
    Real v = y_vals[i] + sin(10 * T) * 0.5;
    mesh.MarkDirichletBoundary(iv, 1, v);
  }
}

void handle_armadillo_extreme(fem::LinearMesh<3>& mesh, Real T) {
  using namespace ax::math;
  static std::vector<Index> l_dirichlet_handles;
  static std::vector<Index> r_dirichlet_handles;
  static std::vector<Real> x_vals_l, x_vals_r;
  if (l_dirichlet_handles.empty()) {
    // first time call.
    RealVector3 center{0.7, 0.6, 0.6};
    Real radius = 0.1;
    for (Index i = 0; i < mesh.GetNumVertices(); ++i) {
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
    RealVector3 center{-0.7, 0.77, 0.4};
    Real radius = 0.1;
    for (Index i = 0; i < mesh.GetNumVertices(); ++i) {
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
    Index iv = l_dirichlet_handles[i];
    Real v = x_vals_l[i] + T * 0.5;
    mesh.MarkDirichletBoundary(iv, 0, v);
  }
  for (size_t i = 0; i < r_dirichlet_handles.size(); ++i) {
    Index iv = r_dirichlet_handles[i];
    Real v = x_vals_r[i] - T * 0.5;
    mesh.MarkDirichletBoundary(iv, 0, v);
  }
}

void ui_callback(gl::UiRenderEvent) {
  static Index frame = 0;
  static bool save_result = false;
  ImGui::Begin("FEM", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
  ImGui::Checkbox("Running", &running);
  ImGui::Text("dt=%lf", dt);
  ImGui::Text("#Elements %ld, #Vertices %ld", ts->GetMesh()->GetNumElements(),
              ts->GetMesh()->GetNumVertices());
  ImGui::Checkbox("Save Result", &save_result);
  static float drop_ratio = 0.1;
  ImGui::InputFloat("drop ratio", &drop_ratio);
  if (ImGui::Button("Step") || running) {
    const auto& vert = ts->GetMesh()->GetVertices();
    if (save_result) {
      static size_t cnt = 0;
      static std::once_flag flag;
      std::call_once(flag, [&]() {
        math::RealMatrixX x0 = ts->GetMesh()->GetVertices();
        math::write_npy_v10(fmt::format("outs/initial.npy"), x0);
      });

      const auto& traj = ts->GetLastTrajectory();
      Real g0 = 0;
      for (const auto& [i, u] : utils::views::enumerate(traj) | utils::views::drop_last(1)) {
        // Randomly drop some.
        float r = (rand() % 128) / 128.0f;
        if (drop_ratio > r) {
          continue;
        }

        math::RealMatrixX g = ts->Gradient(u);
        if (g.norm() < 0.001 * g0 && i > 0) {
          break;
        }
        if (i == 0) {
          g0 = g.norm();
        }
        math::write_npy_v10(fmt::format("outs/grad_{}.npy", cnt), g);
        math::RealMatrixX du = u - traj[i+1];
        // std::cout << math::dot(g, du) << std::endl;
        math::write_npy_v10(fmt::format("outs/pos_{}.npy", cnt), du);
        ++cnt;
      }
    }
    if (scene == SCENE_TWIST) {
      // Apply some Dirichlet BC
      math::RealMatrix3 rotate = Eigen::AngleAxis<Real>(dt, math::RealVector3::UnitX()).matrix();
      for (auto i : utils::range(vert.cols())) {
        const auto& position = vert.col(i);
        if (-position.x() > 4.9) {
          // Mark as dirichlet bc.
          math::RealVector3 p = rotate * position;
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

    auto time_start = utils::get_current_time_nanos();
    ts->BeginTimestep();
    ts->SolveTimestep();
    ts->EndTimestep();
    auto time_end = utils::get_current_time_nanos();
    auto time_elapsed = (time_end - time_start) * 1e-9;
    fps[frame++ % fps.size()] = 1.0 / time_elapsed;
    ;
    AX_INFO("Dt={}s, FPS={}", time_elapsed, fps.sum() / std::min<Index>(100, frame));
    update_rendering();
  }

  static bool on_x = false, on_y = false, on_z = false;
  static float ratio = 0.;
  bool changed = ImGui::SliderFloat("Ratio", &ratio, -1, 1);
  static int selected_row = 0;
  changed |= ImGui::Checkbox("Apply x", &on_x);
  changed |= ImGui::Checkbox("Apply y", &on_y);
  changed |= ImGui::Checkbox("Apply z", &on_z);
  changed |= ImGui::InputInt("Selected Row", &selected_row);
  selected_row = std::clamp<int>(selected_row, 0, g_basis.cols());

  if (changed) {
    patch_component<gl::Mesh>(out, [&](gl::Mesh& m) {
      m.vertices_ = ts->GetPosition();
      if (on_x) {
        m.vertices_.row(0) += g_basis.row(selected_row) * ratio;
      }
      if (on_y) {
        m.vertices_.row(1) += g_basis.row(selected_row) * ratio;
      }
      if (on_z) {
        m.vertices_.row(2) += g_basis.row(selected_row) * ratio;
      }
    });
  }
  ImGui::End();
}

void fix_negative_volume(math::IndexField4& tets, math::RealField3 const& verts) {
  for (auto i : utils::range(tets.cols())) {
    auto a = verts.col(tets(0, i));
    auto b = verts.col(tets(1, i));
    auto c = verts.col(tets(2, i));
    auto d = verts.col(tets(3, i));
    math::RealMatrix4 tet;
    tet << a, b, c, d, 0, 0, 0, 1;
    if (math::det(tet) < 0) {
      std::swap(tets(1, i), tets(2, i));
    }
  }
}

int main(int argc, char** argv) {
  po::add_option({
    po::make_option<Real>("youngs", "Youngs modulus", "1e7"),
    po::make_option<int>("N", "Num of division", "3"),
    po::make_option("scene", "id of scene, 0 for twist, 1 for bend.", "bend"),
    po::make_option("elast", "Hyperelasticity model, nh=Neohookean arap=Arap", "stable_neohookean"),
    po::make_option("optim", "optimizer newton 'naive' or 'liu'", "newton"),
    po::make_option("device", "cpu or gpu", "gpu"),
    po::make_option<bool>("optopo", "Optimize topology using RCM", "true"),
    po::make_option("lbfgs", "naive, laplacian, hard", "laplacian"),
    po::make_option("ls", "Line searcher", "Backtracking"),
    po::make_option("verbose", "Verbose"),
  });

  gl::init(argc, argv);
  fps.setZero(100);
  auto sname = po::get_parse_result()["scene"].as<std::string>();
  if (sname == "twist") {
    scene = SCENE_TWIST;
  } else if (sname == "bend") {
    scene = SCENE_BEND;
  } else if (sname == "drag") {
    scene = SCENE_ARMADILLO_DRAG;
  } else if (sname == "extreme") {
    scene = SCENE_ARMADILLO_EXTREME;
  } else {
    AX_CHECK(false, "Invalid scene name.");
  }
  nx = po::get_parse_result()["N"].as<int>();

  std::string tet_file, vet_file;

  auto youngs = po::get_parse_result()["youngs"].as<Real>();

  lame = fem::elasticity::compute_lame(youngs, 0.45);
  if (scene == SCENE_TWIST || scene == SCENE_BEND) {
    tet_file = utils::get_asset("/mesh/npy/beam_high_res_elements.npy");
    vet_file = utils::get_asset("/mesh/npy/beam_high_res_vertices.npy");
    // tet_file = utils::get_asset("/mesh/npy/beam_low_res_elements.npy");
    // vet_file = utils::get_asset("/mesh/npy/beam_low_res_vertices.npy");
  } else if (scene == SCENE_ARMADILLO_DRAG || scene == SCENE_ARMADILLO_EXTREME) {
    tet_file = utils::get_asset("/mesh/npy/armadillo_low_res_larger_elements.npy");
    vet_file = utils::get_asset("/mesh/npy/armadillo_low_res_larger_vertices.npy");
  }

  g_basis = math::read_npy_v10_real("/home/adversarr/Repo/neural_subspace/basis.npy").transpose();
  auto tet = math::read_npy_v10_Index(tet_file);
  auto vet = math::read_npy_v10_real(vet_file);
  auto cube = geo::tet_cube(0.5, 10 * nx, nx, nx);
  cube.vertices_.row(0) *= 10;
  input_mesh.indices_ = tet.transpose();
  input_mesh.vertices_ = vet.transpose();
  bool verbose = po::get_parse_result()["verbose"].as<bool>();
  // input_mesh = cube;

  if (auto opt = po::get_parse_result()["optim"].as<std::string>(); opt == "liu") {
    ts = std::make_unique<fem::Timestepper_QuasiNewton<3>>(std::make_shared<fem::LinearMesh<3>>());
    auto strategy = po::get_parse_result()["lbfgs"].as<std::string>();
    auto* p_ts = reinterpret_cast<fem::Timestepper_QuasiNewton<3>*>(ts.get());
    if (strategy == "naive") {
      std::cout << "LBFGS: Naive" << std::endl;
      p_ts->SetOptions({{"lbfgs_strategy", "kNaive"}});
    } else if (strategy == "laplacian") {
      std::cout << "LBFGS: Liu17" << std::endl;
      p_ts->SetOptions({{"lbfgs_strategy", "kLaplacian"}});
    } else if (strategy == "hard") {
      std::cout << "LBFGS: Hard" << std::endl;
      p_ts->SetOptions({{"lbfgs_strategy", "kHard"}});
    } else {
      p_ts->SetOptions({{"lbfgs_strategy", "kReservedForExperimental"}});
    }
  } else if (opt == "newton") {
    std::cout << "Newton" << std::endl;
    ts = std::make_unique<fem::Timestepper_NaiveOptim<3>>(std::make_shared<fem::LinearMesh<3>>());
    ts->SetOptions({{"optimizer_opt", utils::Options{{"verbose", true}}}});
  } else {
    AX_CHECK(false, "Invalid optimizer name, expect 'liu' or 'newton', got {}", opt);
  }
  ts->SetYoungs(youngs);
  ts->SetPoissonRatio(0.45);

  ts->GetMesh()->SetMesh(input_mesh.indices_, input_mesh.vertices_);
  ts->GetMesh()->SetNumDofPerVertex(3);
  ts->GetMesh()->ResetAllBoundaries();

  input_mesh.vertices_ = ts->GetMesh()->GetVertices();
  input_mesh.indices_ = ts->GetMesh()->GetElements();
  if (scene == SCENE_TWIST || scene == SCENE_BEND) {
    auto vertices_accessor = math::make_accessor(input_mesh.vertices_);
    for (auto [i, position] : math::enumerate(vertices_accessor)) {
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
  ts->SetupElasticity(po::get_parse_result()["elast"].as<std::string>(),
#ifdef AX_HAS_CUDA
                      po::get_parse_result()["device"].as<std::string>()
#else
                      "cpu"
#endif
  );
  std::string ls = po::get_parse_result()["ls"].as<std::string>();
  ts->SetOptions({
    {
      "optimizer_opt",
      utils::Options{
        {"linesearch", ls},
        {"verbose", verbose},
      },
    },
    {"verbose", verbose},
    {"record_trajectory", true},
  });

  ts->Initialize();

  AX_WARN("Timestepper Options: {}", boost::json::value(ts->GetOptions()));

  if (scene == SCENE_TWIST || scene == SCENE_ARMADILLO_DRAG || scene == SCENE_ARMADILLO_EXTREME) {
    ts->SetExternalAcceleration(math::RealField3::Zero(3, ts->GetMesh()->GetNumVertices()));
  } else {
    ts->SetExternalAccelerationUniform(math::RealVector3{0, -9.8, 0});
  }

  ts->BeginSimulation(dt);
  out = create_entity();
  add_component<gl::Mesh>(out);
  add_component<gl::Lines>(out);
  connect<gl::UiRenderEvent, &ui_callback>();
  get_resource<gl::Context>().Initialize();

  update_rendering();
  gl::enter_main_loop();

  ts.reset();
  clean_up();
  return 0;
}
