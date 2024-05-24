#include <imgui.h>
#include <chrono>
#include <ratio>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/geometry/primitives.hpp"
#include "ax/gl/events.hpp"
#include "ax/gl/primitives/lines.hpp"
#include "ax/gl/utils.hpp"
#include "ax/math/common.hpp"
#include "ax/math/io.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/utils/iota.hpp"
#include "ax/xpbd/common.hpp"
#include "ax/xpbd/constraints/hard.hpp"
#include "ax/xpbd/constraints/plane_collider.hpp"
#include "ax/xpbd/constraints/tet.hpp"

using namespace ax;

Entity ent;
ABSL_FLAG(int, nx, 2, "cloth resolution");
bool has_adaptive_rho = false;

math::vec3r ball_center{0, -1, 0};
real ball_radius = 0.5;
real ground_z = -1;
xpbd::Constraint_PlaneCollider* sc;
void update_rendering() {
  auto& lines = add_or_replace_component<gl::Lines>(ent);
  auto& g = xpbd::ensure_server();
  lines.vertices_ = g.vertices_;
  List<std::pair<idx, idx>> edges;
  for (auto const& c : g.constraints_) {
    idx nC = c->GetNumConstraints();
    auto &ids = c->GetConstrainedVerticesIds();
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
    if (has_adaptive_rho) {
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
    auto begin_time = std::chrono::high_resolution_clock::now();
    step();
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - begin_time);
    std::cout << "Elapsed time: " << elapsed.count() / 1000.0 << " ms" << std::endl;
    update_rendering();
  }

  ImGui::InputDouble("ground value", &sc->offset_);
  ImGui::End();
}

int main(int argc, char** argv) {
  gl::init(argc, argv);
  connect<gl::UiRenderEvent, &ui_callback>();
  ent = create_entity();
  auto& g = xpbd::ensure_server();
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kTetra));
  auto* sp = reinterpret_cast<xpbd::Constraint_Tetra*>(g.constraints_.back().get());
  int ndiv = absl::GetFlag(FLAGS_nx);
  // auto cube = geo::tet_cube(0.5, ndiv, ndiv, ndiv);
  auto cube = geo::TetraMesh();
  cube.vertices_ = math::read_npy_v10_real(utils::get_asset("/mesh/npy/beam_mid_res_vertices.npy")).transpose();
  cube.indices_ = math::read_npy_v10_idx(utils::get_asset("/mesh/npy/beam_mid_res_elements.npy")).transpose();
  auto const& vertices = cube.vertices_;
  g.vertices_ = vertices;
  g.vertices_.row(1).array() += 1;
  g.velocities_.setZero(3, vertices.cols());
  g.ext_accel_ = g.velocities_;
  g.ext_accel_.row(1).setConstant(-9.8);
  g.mass_.setConstant(1, vertices.cols(), 1e-3);
  math::field1r stiff;
  stiff.setConstant(1, cube.indices_.size(), 3e5);
  sp->SetTetrahedrons(cube.indices_, stiff);

  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kInertia));
 
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kPlaneCollider));
  sc = reinterpret_cast<xpbd::Constraint_PlaneCollider*>(g.constraints_.back().get());
  g.dt_ = 1e-2;
  g.constraints_.emplace_back(xpbd::ConstraintBase::Create(xpbd::ConstraintKind::kBallCollider));
  update_rendering();
  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return EXIT_SUCCESS;
}