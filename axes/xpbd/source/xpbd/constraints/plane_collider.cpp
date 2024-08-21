#include "ax/xpbd/constraints/plane_collider.hpp"

#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/ndrange.hpp"

namespace ax::xpbd {

static math::RealVector3 relax_ipc(Real rho, Real& k, Real eps, math::RealVector3 const& u,
                             math::RealVector3 const& z, math::RealVector3 const& normal, Real offset) {
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
  Real beta = normal.dot(z - u);
  if (k * eps < (offset - beta) * rho) {
    k = 4 * (offset - beta) * rho / eps;  // enlarger k to satisfy the constraint strictly.
  }
  Real alpha = k * (offset + eps - beta) / (k + rho);
  return alpha * normal + z - u;
}

ConstraintSolution Constraint_PlaneCollider::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  Index const nC = this->GetNumConstraints();
  ConstraintSolution sol(nC);

  for (auto i : utils::range(nC)) {
    Real rho = this->rho_[i];
    Real& k = stiffness_[i];
    math::RealVector3 const& z = constrained_vertices_position_[i];
    math::RealVector3 dual_new = relax_ipc(rho, k, tol_, gap_[i], z, normal_, offset_);
    sol.sqr_dual_residual_ += math::norm2(dual_new - dual_[i]);
    dual_[i] = dual_new;
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

Real Constraint_PlaneCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  Real sqr_prim_res = 0;
  for (auto i : utils::range(this->GetNumConstraints())) {
    math::RealVector3 du = dual_[i] - fetch_from_global[i];
    gap_[i] += du;
    // std::cout << "gap[" << i << "] = " << gap_[i].transpose() << std::endl;
    sqr_prim_res += math::norm2(du);
  }
  return sqr_prim_res;
}

void Constraint_PlaneCollider::EndStep() {}

void Constraint_PlaneCollider::UpdateRhoConsensus(Real scale) {
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_PlaneCollider::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<Index> new_vertices;
  for (Index i : utils::range(g.vertices_.cols())) {
    math::RealVector3 const x = g.vertices_.col(i);
    if (normal_.dot(x) < offset_ + tol_) {
      if (collidings_.find(i) == collidings_.end()) {
        collidings_.insert(i);
        new_vertices.push_back(i);
      }
    }
  }
  if (new_vertices.size() > 0) {
    for (Index i : utils::range(new_vertices.size())) {
      Index iV = new_vertices[i];
      this->constrained_vertices_ids_.push_back(iV);
      this->constraint_mapping_.emplace_back(iV);
      this->constrained_vertices_position_.push_back(g.vertices_.col(iV));
      dual_.push_back(g.vertices_.col(iV));
      gap_.push_back(math::RealVector3::Zero());
      stiffness_.push_back(initial_rho_ * g.dt_ * g.dt_);
      this->rho_.push_back(initial_rho_ * g.dt_ * g.dt_);
    }
  }

  Index n_v = this->GetNumConstrainedVertices();
  for (Index i : utils::range(n_v)) {
    Index iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd