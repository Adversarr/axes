#include <imgui.h>

#include <cinttypes>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/events.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/primitives/quiver.hpp"
#include "ax/gl/utils.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/iota.hpp"
#include "ax/xpbd/common.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/inertia.hpp"
#include "ax/xpbd/constraints/spring.hpp"

using namespace ax;

Entity ent;
ABSL_FLAG(int, nx, 4, "cloth resolution");

void update_rendering() {
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
  lines.colors_.setOnes(4, edges.size());
  lines.flush_ = true;

  auto& particles = add_or_replace_component<gl::Mesh>(ent);
  auto ball = geo::sphere(0.03, 5, 5);
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
  math::field3r const X = g.vertices_;
  g.last_vertices_.swap(g.vertices_);

  // initial guess is inertia position:
  g.vertices_.noalias() = X + g.dt_ * (g.velocities_ + g.dt_ * g.ext_accel_);
  for (auto& c : g.constraints_) {
    c->UpdateRhoConsensus(g.dt_ * g.dt_);
    c->BeginStep();
  }

  math::field1r w(1, X.cols());
  for (idx i = 0; i < n_iter; ++i) {
    g.last_vertices_ = g.vertices_;
    g.vertices_.setZero();
    w.setZero(1, X.cols());
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
    for (idx iV = 0; iV < X.cols(); ++iV) {
      g.vertices_.col(iV) /= w(iV);
    }

    // y_i step:
    for (auto& c : g.constraints_) {
      c->UpdatePositionConsensus();
      real sqr_primal_residual_c = c->UpdateDuality();
      sqr_primal_residual += sqr_primal_residual_c;
      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown")
                   << " R_prim^2=" << sqr_primal_residual_c;
    }

    real scale = 1.0;
    real dual_residual = std::sqrt(sqr_dual_residual);
    real primal_residual = std::sqrt(sqr_primal_residual);
    if (primal_residual > dual_residual * g.primal_dual_threshold_) {
      scale = g.primal_dual_ratio_;
    } else if (dual_residual > primal_residual * g.dual_primal_threshold_) {
      scale = 1.0 / g.dual_primal_ratio_;
    }
    if (scale != 1.0) {
      for (auto& c : g.constraints_) {
        c->UpdateRhoConsensus(scale);
      }
    }
    AX_LOG(WARNING) << i << "===> rho updown: " << scale << " " << primal_residual << " "
                    << dual_residual;
  }

  for (auto& c : g.constraints_) {
    c->EndStep();
  }

  g.velocities_ = (g.vertices_ - X) / 0.01;
}

void ui_callback(gl::UiRenderEvent const&) {
  ImGui::Begin("XPBD");

  static bool running = false;
  ImGui::Checkbox("Running", &running);
  ImGui::InputInt("Iterations", &n_iter);
  if (ImGui::Button("Run Once") || running) {
    step();
    update_rendering();
  }
  ImGui::End();
}

int main(int argc, char** argv) {
  gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_callback>();
  ent = create_entity();
  auto& g = xpbd::ensure_server();
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kSpring));
  auto* sp = reinterpret_cast<xpbd::Constraint_Spring*>(g.constraints_.back().get());

  g.vertices_.setZero(3, 4);
  g.vertices_.col(0).setUnit(0);
  g.vertices_.col(1).setUnit(1);
  g.vertices_.col(2).setUnit(2);
  g.velocities_.setZero(3, g.vertices_.cols());
  g.ext_accel_ = g.velocities_;
  g.velocities_.col(3) = 0.3 * math::vec3r::Ones();
  g.mass_.setConstant(1, g.vertices_.cols(), 1.0);

  math::field2i edges(2, 3);
  edges << 0, 1, 2, 1, 2, 0;
  sp->SetSprings(edges, math::field1r::Constant(1, 3, 1e5));

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kInertia));
  g.dt_ = 5e-3;

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kVertexFaceCollider));

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kHard));
  auto *hard = reinterpret_cast<xpbd::Constraint_Hard*>(g.constraints_.back().get());
  math::field1i hard_indices(1, 1);
  hard_indices << 0;
  hard->SetHard(hard_indices);

  g.faces_.emplace_back(0, 1, 2);


  update_rendering();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}
