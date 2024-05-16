#include <imgui.h>
#include <cinttypes>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/events.hpp"
#include "ax/gl/primitives/lines.hpp"
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
  auto& g = xpbd::ensure_server<3>();
  lines.vertices_ = g.vertices_;
  List<std::pair<idx, idx>> edges;
  for (auto const& c : g.constraints_) {
    idx nC = c->GetNumConstraints();
    edges.reserve(edges.size() + nC);
    for (idx i = 0; i < nC; ++i) {
      auto const& ij = c->GetConstraintMapping().col(i);
      for (idx j = 1; j < ij.rows(); ++j) {
        edges.push_back({ij(j - 1), ij(j)});
      }
    }
  }

  lines.indices_.resize(2, edges.size());
  for (idx i = 0; i < edges.size(); ++i) {
    lines.indices_.col(i) = math::vec2i{edges[i].first, edges[i].second};
  }
  lines.colors_.setOnes(4, edges.size());
  lines.flush_ = true;
}

int n_iter = 2;

void step() {
  auto& g = xpbd::ensure_server<3>();
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

      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown") << " "
                   << R.sqr_dual_residual_;
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
      AX_LOG(INFO) << "Constraint: " << utils::reflect_name(c->GetKind()).value_or("Unknown") << " "
                   << sqr_primal_residual_c;
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
      AX_LOG(ERROR) << i << "===> Updating rho: " << scale << " " << primal_residual << " " << dual_residual;
    }
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
  auto& g = xpbd::ensure_server<3>();
  g.constraints_.emplace_back(xpbd::ConstraintBase<3>::Create(xpbd::ConstraintKind::kSpring));
  auto* sp = reinterpret_cast<xpbd::Constraint_Spring<3>*>(g.constraints_.back().get());
  int ndiv = absl::GetFlag(FLAGS_nx);
  auto springs = math::field2i(2, ndiv * (ndiv - 1) * 2 + (ndiv - 1) * (ndiv - 1));
  auto vertices = math::field3r(3, ndiv * ndiv);
  auto initial = math::field1r(1, springs.cols());
  idx cnt = 0;
  for (idx i = 0; i < ndiv; i++) {
    for (idx j = 0; j < ndiv; j++) {
      vertices.col(cnt) = math::vec3r(i, 0, j) / (ndiv - 1);
      cnt++;
    }
  }
  cnt = 0;
  for (idx i = 0; i < ndiv; i++) {
    for (idx j = 0; j < ndiv; j++) {
      if (i < ndiv - 1) {
        springs.col(cnt) = math::vec2i(i * ndiv + j, (i + 1) * ndiv + j);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col((i + 1) * ndiv + j)).norm();
        cnt++;
      }
      if (j < ndiv - 1) {
        springs.col(cnt) = math::vec2i(i * ndiv + j, i * ndiv + j + 1);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col(i * ndiv + j + 1)).norm();
        cnt++;
      }
      if (i < ndiv - 1 && j < ndiv - 1) {
        springs.col(cnt) = math::vec2i(i * ndiv + j, (i + 1) * ndiv + j + 1);
        initial[cnt] = (vertices.col(i * ndiv + j) - vertices.col((i + 1) * ndiv + j + 1)).norm();
        cnt++;
      }
    }
  }
  g.vertices_ = vertices;
  sp->SetSprings(springs, math::field1r::Constant(1, springs.cols(), 1e4));

  g.velocities_.setZero(3, vertices.cols());
  g.ext_accel_ = g.velocities_;
  g.ext_accel_.row(1).setConstant(-9.8);
  g.mass_.setConstant(1, vertices.cols(), 1.0);

  g.constraints_.emplace_back(xpbd::ConstraintBase<3>::Create(xpbd::ConstraintKind::kInertia));
  g.constraints_.emplace_back(xpbd::ConstraintBase<3>::Create(xpbd::ConstraintKind::kHard));
  auto* hard = reinterpret_cast<xpbd::Constraint_Hard<3>*>(g.constraints_.back().get());
  math::field1i indices_hard(1, 2);
  indices_hard(0, 0) = 0;
  indices_hard(0, 1) = ndiv - 1;
  hard->SetHard(indices_hard);

  g.dt_ = 1e-2;
  update_rendering();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}