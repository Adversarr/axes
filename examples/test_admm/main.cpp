#include <imgui.h>
#include <implot.h>

#include <cinttypes>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/accel/flat_octree.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/events.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/iota.hpp"
#include "ax/xpbd/common.hpp"
#include "ax/xpbd/constraints/colliding_balls.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/inertia.hpp"
#include "ax/xpbd/constraints/plane_collider.hpp"
#include "ax/xpbd/constraints/spring.hpp"
#include "ax/xpbd/constraints/tet.hpp"
#include "ax/xpbd/global_step_collision_free.hpp"

using namespace ax;
using namespace ax::xpbd;

Entity ent;
ABSL_FLAG(int, nx, 4, "cloth resolution");
ABSL_FLAG(real, ball_radius, 0.1, "Radius for balls.");
std::vector<float> running_time;
real R;

// render_aabb:
void render_aabb() {
  static Entity ent = create_entity();
  auto& box = add_or_replace_component<gl::Mesh>(ent);
  std::vector<geo::AlignedBox3> boxes;

  auto& g = xpbd::ensure_server();
  geo::BroadPhase_FlatOctree otree;
  for (idx i = 0; i < g.vertices_.cols(); ++i) {
    geo::AlignedBox3 box;
    box.min() = g.vertices_.col(i);
    box.max() = g.vertices_.col(i);
    box.min().array() -= R;
    box.max().array() += R;
    otree.AddCollider(box, i, i, geo::PrimitiveKind::kVertex);
  }

  otree.DetectCollisions();
  otree.ForeachTreeAABB([&boxes](geo::AlignedBox3 const& aabb) { boxes.push_back(aabb); });

  auto const& cp = otree.GetCollidingPairs();
  size_t total_pot_collision = 0;
  for (auto const& [k, v] : cp) {
    total_pot_collision += v.size();
  }
  AX_LOG(ERROR) << "Potential Collisions: " << total_pot_collision
                << "Total Vertices Pair: " << g.vertices_.cols() * (g.vertices_.cols() - 1) / 2;

  auto cube = geo::cube(.5);
  box.vertices_ = cube.vertices_;
  box.colors_.setConstant(4, cube.vertices_.cols(), 0.7);
  box.indices_ = cube.indices_;
  box.instance_offset_.resize(3, boxes.size());
  box.instance_scale_.resize(3, boxes.size());
  box.is_flat_ = true;
  for (auto&& [i, b] : utils::enumerate(boxes)) {
    box.instance_offset_.col(i) = b.min();
    box.instance_scale_.col(i) = b.sizes();
  }
  box.flush_ = true;
}

void update_rendering() {
  // render_aabb();
  auto& lines = add_or_replace_component<gl::Lines>(ent);
  auto& g = xpbd::ensure_server();
  lines.vertices_ = g.vertices_;
  List<std::pair<idx, idx>> edges;
  for (auto const& c : g.constraints_) {
    idx nC = c->GetNumConstraints();
    auto& ids = c->GetConstrainedVerticesIds();
    edges.reserve(edges.size() + nC);
    for (idx i = 0; i < nC; ++i) {
      auto const& ij = c->GetConstraintMapping()[i];
      for (idx j = 1; j < ij.size(); ++j) {
        for (idx k = 0; k < j; ++k) {
          edges.push_back({ids[ij[k]], ids[ij[j]]});
        }
      }
    }
  }

  lines.indices_.resize(2, edges.size());
  for (idx i = 0; i < edges.size(); ++i) {
    lines.indices_.col(i) = math::vec2i{edges[i].first, edges[i].second};
  }
  lines.colors_.setOnes(4, g.vertices_.size());
  lines.flush_ = true;

  auto& particles = add_or_replace_component<gl::Mesh>(ent);
  auto ball = geo::sphere(R, 16, 16);
  particles.vertices_ = ball.vertices_;
  particles.indices_ = ball.indices_;
  particles.colors_.setOnes(4, ball.vertices_.cols());
  particles.instance_offset_ = g.vertices_;
  particles.flush_ = true;

  // quiver
  auto& quivers = add_or_replace_component<gl::Quiver>(ent);
  quivers.positions_ = g.vertices_;
  quivers.directions_ = g.velocities_;
  quivers.flush_ = true;
  quivers.colors_.setConstant(4, quivers.positions_.cols(), 0.7);
}

int n_iter = 2;

void step() {
  auto& g = xpbd::ensure_server();
  idx const nV = g.vertices_.cols();
  g.last_vertices_.swap(g.vertices_);

  // initial guess is inertia position:
  g.vertices_.noalias() = g.last_vertices_ + g.dt_ * (g.velocities_ + g.dt_ * g.ext_accel_);
  for (auto& c : g.constraints_) {
    c->UpdateRhoConsensus(g.dt_ * g.dt_);
    c->BeginStep();
  }

  math::field1r w(1, nV);
  for (idx i = 0; i < n_iter; ++i) {
    g.vertices_.setZero();
    w.setZero(1, nV);
    real sqr_dual_residual = 0;
    real sqr_primal_residual = 0;
    for (auto& c : g.constraints_) {
      auto R = c->SolveDistributed();
      // x_i step:
      for (auto I : utils::iota(R.weights_.size())) {
        idx iV = c->GetConstrainedVerticesIds()[I];
        g.vertices_.col(iV) += R.weighted_position_.col(I);
        w(iV) += R.weights_[I];
      }
      sqr_dual_residual += R.sqr_dual_residual_;

      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown")
                   << " R_dual^2=" << R.sqr_dual_residual_;
    }

    // z_i step:
    xpbd::global_step_collision_free(g.vertices_, w);

    // y_i step:
    for (auto& c : g.constraints_) {
      c->UpdatePositionConsensus();
      real sqr_primal_residual_c = c->UpdateDuality();
      sqr_primal_residual += sqr_primal_residual_c;
      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown")
                   << " R_prim^2=" << sqr_primal_residual_c;
    }

    // real scale = 1.0;
    // real dual_residual = std::sqrt(sqr_dual_residual);
    // real primal_residual = std::sqrt(sqr_primal_residual);
    // if (primal_residual > dual_residual * g.primal_dual_threshold_) {
    //   scale = g.primal_dual_ratio_;
    // } else if (dual_residual > primal_residual * g.dual_primal_threshold_) {
    //   scale = 1.0 / g.dual_primal_ratio_;
    // }
    // if (scale != 1.0) {
    //   for (auto& c : g.constraints_) {
    //     c->UpdateRhoConsensus(scale);
    //   }
    //   AX_LOG(WARNING) << i << "===> rho updown: " << scale << " " << primal_residual << " "
    //                   << dual_residual;
    // }
  }

  for (auto& c : g.constraints_) {
    c->EndStep();
  }

  g.velocities_ = (g.vertices_ - g.last_vertices_) / g.dt_;
  // std::cout << g.velocities_.rowwise().sum() << std::endl;
}

void ui_callback(gl::UiRenderEvent const&) {
  ImGui::Begin("XPBD");

  static bool running = false;
  static int frame_id = 0;
  ImGui::Checkbox("Running", &running);
  ImGui::InputInt("Iterations", &n_iter);
  if (ImGui::Button("Run Once") || running) {
    auto start = std::chrono::high_resolution_clock::now();
    for (auto _ : utils::iota(10)) {
      step();
    }
    auto end = std::chrono::high_resolution_clock::now();
    // milli seconds
    running_time[frame_id % running_time.size()]
        = std::chrono::duration<double, std::milli>(end - start).count();
    update_rendering();
    frame_id += 1;
  }

  if (ImPlot::BeginPlot("Running Time(ms)")) {
    ImPlot::PlotBars("Running Time", running_time.data(), running_time.size(), 1.0f);
    ImPlot::EndPlot();
  }

  ImGui::End();
}

int main(int argc, char** argv) {
  gl::init(argc, argv);
  R = absl::GetFlag(FLAGS_ball_radius);

  running_time.resize(80);
  connect<gl::UiRenderEvent, &ui_callback>();
  ent = create_entity();
  auto& g = xpbd::ensure_server();
  g.dt_ = 1e-3;
  idx nx = absl::GetFlag(FLAGS_nx);
  auto cube = geo::tet_cube(0.2, 3, 3, 3);
  auto nv_cube = cube.vertices_.cols();

  idx nB = nx;
  idx nV = 5;

  g.vertices_.setZero(3, nV);
  g.velocities_.setZero(3, nV);
  g.last_vertices_ = g.vertices_;
  g.ext_accel_.setZero(3, nV);
  g.vertices_.leftCols<3>().setIdentity();
  g.vertices_.col(3) = math::vec3r{0, 0, -1};
  g.vertices_.col(4) = math::vec3r{.5, 1.00000, 0};
  g.velocities_.col(4) = math::vec3r{0, -1, 0};

  g.faces_.push_back({0, 1, 2});
  g.faces_.push_back({0, 1, 3});

  g.mass_.setConstant(1, nV, 0.001);
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kInertia));
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kVertexFaceCollider));

  update_rendering();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}
