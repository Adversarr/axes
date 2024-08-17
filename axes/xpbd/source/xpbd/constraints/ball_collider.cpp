#include "ax/xpbd/constraints/ball_collider.hpp"

#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

template <typename T> void resize_and_copyback(T& v, Index r, Index c) {
  T old = v;
  v.resize(r, c);
  v.topLeftCorner(old.rows(), old.cols()) = old;
}

static math::RealVector3 relax_ipc(real rho, real& k, real eps, math::RealVector3 const& u,
                             math::RealVector3 const& z, math::RealVector3 const& center, real radius) {
  // Very similar to spring, let L = radius + eps.
  // The spring energy:
  //   f(x) = 1/2 k (|| x - c || - L)^2
  // with Augmented Lagrangian:
  //   f(x) + 1/2 rho || x - z + u ||^2
  // Take derivative:
  //   k (|x-c| - L) (x-c)/|x-c| + rho (x - z + u) = 0
  // let x = c + alpha normalized(z - u - c) = c + alpha n.
  //   k (alpha - L) n + rho (c + alpha n - z + u) = 0
  //   k (alpha - L) n + rho alpha n = rho (z - u - c)
  //   k (alpha - L) + rho alpha     = rho |z - u - c|
  //   alpha = (k L + rho |z - u - c|) / (k + rho)
  // we expect k to satisfy
  //   alpha >= radius
  //   (k + rho) radius <= (k (radius + eps) + rho |z - u - c|)
  //   k eps >= rho (|z - u - c| - radius)

  math::RealVector3 zuc = z - u - center;
  real zuc_norm = zuc.norm();
  if (k * eps < rho * (zuc_norm - radius)) {
    k = 4 * rho * (zuc_norm - radius) / eps;
  }
  real alpha = (k * (radius + eps) + rho * zuc_norm) / (k + rho);
  return center + alpha * zuc.normalized();
}

ConstraintSolution Constraint_BallCollider::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  Index const nC = this->GetNumConstraints();
  ConstraintSolution sol(nC);

  for (auto i : utils::iota(nC)) {
    real& rho = this->rho_[i];
    real& k = stiffness_[i];
    math::RealVector3 const& z = constrained_vertices_position_[i];
    dual_[i] = relax_ipc(rho, k, tol_, gap_[i], z, center_, radius_);
    sol.weighted_position_.col(i) += rho * (gap_[i] + dual_[i]);
    sol.weights_[i] += rho;
  }

  iteration_ += 1;
  return sol;
}

void Constraint_BallCollider::BeginStep() {
  gap_.clear();
  dual_.clear();
  collidings_.clear();
  this->constrained_vertices_ids_.clear();
  this->constraint_mapping_.clear();
  this->constrained_vertices_position_.clear();
  this->rho_.clear();
  this->rho_global_ = 1;
  iteration_ = 0;
}

real Constraint_BallCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i : utils::iota(this->GetNumConstraints())) {
    math::RealVector3 du = dual_[i] - fetch_from_global[i];
    gap_[i] += du;
    sqr_prim_res += math::norm2(du);
  }
  return sqr_prim_res;
}

void Constraint_BallCollider::EndStep() {}

void Constraint_BallCollider::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_BallCollider::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<Index> new_vertices;
  for (Index i : utils::iota(g.vertices_.cols())) {
    math::RealVector3 const x = g.vertices_.col(i);
    if (math::norm(x - center_) < radius_ + tol_) {
      if (collidings_.find(i) == collidings_.end()) {
        collidings_.insert(i);
        new_vertices.push_back(i);
      }
    }
  }

  if (new_vertices.size() > 0) {
    for (auto [i, iV] : utils::enumerate(new_vertices)) {
      constrained_vertices_ids_.push_back(iV);
      constraint_mapping_.emplace_back(iV);
      constrained_vertices_position_.push_back(g.vertices_.col(iV));
      dual_.push_back(g.vertices_.col(iV));
      gap_.push_back(math::RealVector3::Zero());
      stiffness_.push_back(initial_rho_ * g.dt_ * g.dt_);
      rho_.push_back(initial_rho_ * g.dt_ * g.dt_);
    }
  }

  Index n_v = this->GetNumConstrainedVertices();
  for (Index i : utils::iota(n_v)) {
    Index iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd