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
#include "ax/utils/ndrange.hpp"
#include "ax/xpbd/common.hpp"
#include "ax/xpbd/constraints/colliding_balls.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/inertia.hpp"
#include "ax/xpbd/constraints/plane_collider.hpp"
#include "ax/xpbd/constraints/spring.hpp"
#include "ax/xpbd/constraints/tet.hpp"
#include "ax/xpbd/global_step_collision_free.hpp"
#include <range/v3/view/enumerate.hpp>

using namespace ax;
xpbd::Constraint_PlaneCollider* bottom;
Entity ent;
ABSL_FLAG(int, nx, 4, "cloth resolution");
ABSL_FLAG(Real, ball_radius, 0.1, "Radius for balls.");
std::vector<float> running_time;
Real R;

// render_aabb:
void render_aabb() {
  static Entity ent = create_entity();
  auto& box = add_or_replace_component<gl::Mesh>(ent);
  std::vector<geo::AlignedBox3> boxes;

  auto& g = xpbd::ensure_server();
  geo::BroadPhase_FlatOctree otree;
  for (Index i = 0; i < g.vertices_.cols(); ++i) {
    geo::AlignedBox3 box;
    box.min() = g.vertices_.col(i);
    box.max() = g.vertices_.col(i);
    box.min().array() -= R;
    box.max().array() += R;
    otree.AddCollider(box, i, i, geo::PrimitiveKind::Vertex);
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
  for (auto&& [i, b] : utils::views::enumerate(boxes)) {
    box.instance_offset_.col(i) = b.min();
    box.instance_scale_.col(i) = b.sizes();
  }
  box;
}

void update_rendering() {
  // render_aabb();
  auto& lines = add_or_replace_component<gl::Lines>(ent);
  auto& g = xpbd::ensure_server();
  lines.vertices_ = g.vertices_;
  std::vector<std::pair<Index, Index>> edges;
  for (auto const& c : g.constraints_) {
    Index nC = c->GetNumConstraints();
    auto& ids = c->GetConstrainedVerticesIds();
    edges.reserve(edges.size() + nC);
    for (Index i = 0; i < nC; ++i) {
      auto const& ij = c->GetConstraintMapping()[i];
      for (Index j = 1; j < ij.size(); ++j) {
        for (Index k = 0; k < j; ++k) {
          edges.push_back({ids[ij[k]], ids[ij[j]]});
        }
      }
    }
  }

  lines.indices_.resize(2, edges.size());
  for (Index i = 0; i < edges.size(); ++i) {
    lines.indices_.col(i) = math::IndexVector2{edges[i].first, edges[i].second};
  }
  lines.colors_.setOnes(4, g.vertices_.size());
  lines;

  auto& particles = add_or_replace_component<gl::Mesh>(ent);
  auto ball = geo::sphere(R, 16, 16);
  particles.vertices_ = ball.vertices_;
  particles.indices_ = ball.indices_;
  particles.colors_.setOnes(4, ball.vertices_.cols());
  particles.instance_offset_ = g.vertices_;
  particles;

  // quiver
  auto& quivers = add_or_replace_component<gl::Quiver>(ent);
  quivers.positions_ = g.vertices_;
  quivers.directions_ = g.velocities_;
  quivers;
  quivers.colors_.setConstant(4, quivers.positions_.cols(), 0.7);
}

int n_iter = 2;

void step() {
  auto& g = xpbd::ensure_server();
  Index const nV = g.vertices_.cols();
  g.last_vertices_.swap(g.vertices_);

  // initial guess is inertia position:
  g.vertices_.noalias() = g.last_vertices_ + g.dt_ * (g.velocities_ + g.dt_ * g.ext_accel_);
  for (auto& c : g.constraints_) {
    c->UpdateRhoConsensus(g.dt_ * g.dt_);
    c->BeginStep();
  }

  math::RealField1 w(1, nV);
  for (Index i = 0; i < n_iter; ++i) {
    g.vertices_.setZero();
    w.setZero(1, nV);
    Real sqr_dual_residual = 0;
    Real sqr_primal_residual = 0;
    for (auto& c : g.constraints_) {
      auto R = c->SolveDistributed();
      // x_i step:
      for (auto I : utils::range(R.weights_.size())) {
        Index iV = c->GetConstrainedVerticesIds()[I];
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
      Real sqr_primal_residual_c = c->UpdateDuality();
      sqr_primal_residual += sqr_primal_residual_c;
      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown")
                   << " R_prim^2=" << sqr_primal_residual_c;
    }

    // Real scale = 1.0;
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
}

void ui_callback(gl::UiRenderEvent const&) {
  ImGui::Begin("XPBD");

  static bool running = false;
  static int frame_id = 0;
  ImGui::Checkbox("Running", &running);
  ImGui::InputInt("Iterations", &n_iter);

  ImGui::InputDouble("Ground Z: ", &bottom->offset_);
  if (ImGui::Button("Run Once") || running) {
    auto start = std::chrono::high_resolution_clock::now();
    for (auto _ : utils::range(10)) {
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
  Index nx = absl::GetFlag(FLAGS_nx);
  auto cube = geo::tet_cube(0.2, 3, 3, 3);
  auto nv_cube = cube.vertices_.cols();

  Index nB = nx;
  Index nV = nB + nv_cube;

  // auto bd_face = geo::get_boundary_triangles(cube.vertices_, cube.indices_);
  // for (Index i = 0; i < bd_face.cols(); ++i) {
  //   g.faces_.emplace_back(bd_face.col(i));
  // }

  g.vertices_.setRandom(3, nV);
  g.vertices_ *= 0.5;
  g.vertices_.leftCols(nv_cube) = cube.vertices_;
  g.vertices_.leftCols(nv_cube).row(1).array() += 1;

  g.velocities_.setZero(3, nV);
  g.last_vertices_ = g.vertices_;
  g.ext_accel_.setZero(3, nV);

  g.mass_.setConstant(1, nV, 0.01);
  g.ext_accel_.row(1).array() -= 9.8;
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::Inertia));

  // g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kTetra));
  // auto* tetra = reinterpret_cast<xpbd::Constraint_Tetra*>(g.constraints_.back().get());
  // tetra->SetTetrahedrons(cube.indices_, math::RealField1::Constant(1, cube.indices_.cols(), 1e5));

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::CollidingBalls));
  auto* cb = reinterpret_cast<xpbd::Constraint_CollidingBalls*>(g.constraints_.back().get());
  cb->ball_radius_ = R * 2;

  // g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kVertexFaceCollider));

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::PlaneCollider));
  bottom = reinterpret_cast<xpbd::Constraint_PlaneCollider*>(g.constraints_.back().get());
  bottom->normal_ = math::RealVector3{0, 1, 0};
  bottom->offset_ = -.5;

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::PlaneCollider));
  auto* left = reinterpret_cast<xpbd::Constraint_PlaneCollider*>(g.constraints_.back().get());
  left->normal_ = math::RealVector3{1, 0, 0};
  left->offset_ = -.5;

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::PlaneCollider));
  auto* right = reinterpret_cast<xpbd::Constraint_PlaneCollider*>(g.constraints_.back().get());
  right->normal_ = math::RealVector3{-1, 0, 0};
  right->offset_ = -.5;

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::PlaneCollider));
  auto* front = reinterpret_cast<xpbd::Constraint_PlaneCollider*>(g.constraints_.back().get());
  front->normal_ = math::RealVector3{0, 0, 1};
  front->offset_ = -.5;

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::PlaneCollider));
  auto* back = reinterpret_cast<xpbd::Constraint_PlaneCollider*>(g.constraints_.back().get());
  back->normal_ = math::RealVector3{0, 0, -1};
  back->offset_ = -.5;

  update_rendering();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}
