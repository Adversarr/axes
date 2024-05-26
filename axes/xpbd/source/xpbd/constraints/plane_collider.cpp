#include "ax/xpbd/constraints/plane_collider.hpp"

#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

static math::vec3r relax_ipc(real rho, real& k, real eps, math::vec3r const& u,
                             math::vec3r const& z, math::vec3r const& normal, real offset) {
  //    E(x) = k/2 (n dot x - offset - eps)^2 + rho/2 || x - z + u ||^2
  //    D E(x) = k (n dot x - offset - eps) n + rho (x - z + u) == 0
  // Let x = alpha n + z - u, and let n dot (z - u) = beta:
  //    k (n dot (alpha n + z - u) - offset - eps) n + rho (alpha n + z - u - z + u) == 0
  // Dot product with n, and use n dot n = 1:
  //    k (alpha - offset - eps + beta) + rho alpha == -k beta
  //    alpha = k (offset + eps - beta) / (k + rho)
  //    x = alpha n + (z - u)
  // but we need n dot x >= offset, therefore we need to adjust k if necessary.
  //    n dot x = alpha + n dot (z - u) >= offset => alpha + beta >= offset.
  // => k (offset + eps - beta) / (k + rho) >= offset - beta.
  // => k (offset + eps - beta) >= (offset - beta) (k + rho)
  // => k eps >= (offset - beta) rho
  real beta = normal.dot(z - u);
  if (k * eps < (offset - beta) * rho) {
    k = 4 * (offset - beta) * rho / eps;  // enlarger k to satisfy the constraint strictly.
  }
  real alpha = k * (offset + eps - beta) / (k + rho);
  return alpha * normal + z - u;
}

ConstraintSolution Constraint_PlaneCollider::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  idx const nC = this->GetNumConstraints();
  ConstraintSolution sol(nC);

  for (auto i : utils::iota(nC)) {
    real rho = this->rho_[i];
    real& k = stiffness_[i];
    math::vec3r const& z = constrained_vertices_position_[i];
    dual_[i] = relax_ipc(rho, k, tol_, gap_[i], z, normal_, offset_);
    // std::cout << "z[" << i << "] = " << z.transpose() << std::endl;
    // std::cout << "dual[" << i << "] = " << dual_[i].transpose() << std::endl;
    sol.weighted_position_.col(i) += rho * (gap_[i] + dual_[i]);
    sol.weights_[i] += rho;
  }

  iteration_ += 1;
  return sol;
}

void Constraint_PlaneCollider::BeginStep() {
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

real Constraint_PlaneCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i : utils::iota(this->GetNumConstraints())) {
    math::vec3r du = dual_[i] - fetch_from_global[i];
    gap_[i] += du;
    // std::cout << "gap[" << i << "] = " << gap_[i].transpose() << std::endl;
    sqr_prim_res += math::norm2(du);
  }
  return sqr_prim_res;
}

void Constraint_PlaneCollider::EndStep() {}

void Constraint_PlaneCollider::UpdateRhoConsensus(real scale) {
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_PlaneCollider::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<idx> new_vertices;
  for (idx i : utils::iota(g.vertices_.cols())) {
    math::vec3r const x = g.vertices_.col(i);
    if (normal_.dot(x) < offset_ + tol_) {
      if (collidings_.find(i) == collidings_.end()) {
        collidings_.insert(i);
        new_vertices.push_back(i);
      }
    }
  }
  if (new_vertices.size() > 0) {
    for (idx i : utils::iota(new_vertices.size())) {
      idx iV = new_vertices[i];
      this->constrained_vertices_ids_.push_back(iV);
      this->constraint_mapping_.emplace_back(iV);
      this->constrained_vertices_position_.push_back(g.vertices_.col(iV));
      dual_.push_back(g.vertices_.col(iV));
      gap_.push_back(math::vec3r::Zero());
      stiffness_.push_back(initial_rho_ * g.dt_ * g.dt_);
      this->rho_.push_back(initial_rho_ * g.dt_ * g.dt_);
    }
  }

  idx n_v = this->GetNumConstrainedVertices();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd