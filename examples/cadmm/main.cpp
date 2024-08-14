#include <imgui.h>

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
#include "ax/xpbd/constraints/ball_collider.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/inertia.hpp"
#include "ax/xpbd/constraints/spring.hpp"
#include "ax/xpbd/global_step_collision_free.hpp"

using namespace ax;

Entity ent;
ABSL_FLAG(int, nx, 4, "cloth resolution");

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
    box.min().array() -= 0.1;
    box.max().array() += 0.1;
    otree.AddCollider(box, i, i, geo::PrimitiveKind::kVertex);
  }
  
  otree.DetectCollisions();
  otree.ForeachTreeAABB([&boxes](geo::AlignedBox3 const& aabb) { boxes.push_back(aabb); });

  auto const& collision_info = otree.GetCollidingPairs();
  for (auto const& [k, v]: collision_info) {
    std::cout << "CollisionType: " << utils::reflect_name(k).value_or("Unknown?") << std::endl;
    for (auto const& info: v) {
      std::cout << "- " << info.a_ << " " << info.b_ << std::endl;
    }
  }
}


void update_rendering() {
  // render_aabb();
  auto& lines = add_or_replace_component<gl::Lines>(ent);
  auto& g = xpbd::ensure_server();
  lines.vertices_ = g.vertices_;
  std::vector<std::pair<idx, idx>> edges;
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
  lines;

  auto& particles = add_or_replace_component<gl::Mesh>(ent);
  auto ball = geo::sphere(0.03, 8, 8);
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
    // for (idx iV = 0; iV < nV; ++iV) {
    //   g.vertices_.col(iV) /= w(iV);
    // }
    xpbd::global_step_collision_free(g.vertices_, w);

    // y_i step:
    for (auto& c : g.constraints_) {
      c->UpdatePositionConsensus();
      real sqr_primal_residual_c = c->UpdateDuality();
      sqr_primal_residual += sqr_primal_residual_c;
      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown")
                   << " R_prim^2=" << sqr_primal_residual_c;
    }
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
  ImGui::Checkbox("Running", &running);
  ImGui::InputInt("Iterations", &n_iter);
  if (ImGui::Button("Run Once") || running) {
    
    auto begin_time = std::chrono::high_resolution_clock::now();
    step();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time);
    std::cout << "Elapsed time: " << elapsed.count() / 1000.0 << " ms" << std::endl;
    update_rendering();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_callback>();
  ent = create_entity();
  auto& g = xpbd::ensure_server();
  g.dt_ = 1e-2;
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kSpring));
  auto* sp = reinterpret_cast<xpbd::Constraint_Spring*>(g.constraints_.back().get());
  idx nx = absl::GetFlag(FLAGS_nx);
  auto plane = geo::plane(0.5, 0.5, nx, nx);

  plane.vertices_.row(2) = plane.vertices_.row(1);
  plane.vertices_.row(1).setZero();
  // idx nB = nx * nx;
  idx nV = plane.vertices_.cols();

  g.vertices_.setZero(3, nV);
  g.vertices_.block(0, 0, 3, plane.vertices_.cols()) = plane.vertices_;
  // g.vertices_.rightCols<1>() = math::vec3r{0.2, 0.1, 0.3};
  // g.vertices_.rightCols(nB).setRandom();
  // g.vertices_.rightCols(nB) *= 0.3;
  // g.vertices_.rightCols(nB).row(1).setConstant(3);

  g.velocities_.setZero(3, g.vertices_.cols());
  g.ext_accel_ = g.velocities_;
  g.ext_accel_.row(1).setConstant(-9.8);
  g.mass_.setConstant(1, g.vertices_.cols(), 1e-3);

  math::field2i edges = geo::get_edges(plane.indices_);
  sp->SetSprings(edges, math::field1r::Constant(1, edges.cols(), 1e4));

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kInertia));

  // g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kVertexFaceCollider));
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kBallCollider));
  auto* bc = reinterpret_cast<xpbd::Constraint_BallCollider*>(g.constraints_.back().get());
  bc->center_ = math::vec3r{0, -1, 0};
  bc->radius_ = 0.4;

  // g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kHard));
  // auto *hard = reinterpret_cast<xpbd::Constraint_Hard*>(g.constraints_.back().get());
  // math::field1i hard_indices = math::field1i::Zero(1, 4);
  // idx cnt = 0;
  // hard_indices << 0, nx, nx * (nx + 1), nx * nx + 2 * nx;
  // hard->SetHard(hard_indices);

  for (auto const& t: math::each(plane.indices_)) {
    g.faces_.push_back(t);
  }

  update_rendering();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}
