#include "ax/xpbd/constraints/ball_collider.hpp"

#include "ax/math/common.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

template <typename T> void resize_and_copyback(T& v, idx r, idx c) {
  T old = v;
  v.resize(r, c);
  v.topLeftCorner(old.rows(), old.cols()) = old;
}

template <idx dim> math::vecr<dim> relax_ipc(real rho, real& k, real eps, math::vecr<dim> const& u,
                                             math::vecr<dim> const& z,
                                             math::vecr<dim> const& center, real radius) {
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

  math::vecr<dim> zuc = z - u - center;
  real zuc_norm = zuc.norm();
  if (k * eps < rho * (zuc_norm - radius)) {
    k = 4 * rho * (zuc_norm - radius) / eps;
  }
  real alpha = (k * (radius + eps) + rho * zuc_norm) / (k + rho);
  return center + alpha * zuc.normalized();
}

template <idx dim> ConstraintSolution<dim> Constraint_BallCollider<dim>::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  idx const nC = this->GetNumConstraints();
  ConstraintSolution<dim> sol(nC);

  for (auto i : utils::iota(nC)) {
    real rho = this->rho_[i];
    real& k = stiffness_[i];
    math::vecr<dim> const& z = this->constrained_vertices_position_[i];
    dual_[i] = relax_ipc<dim>(rho, k, tol_, gap_[i], z, center_, radius_);
    // std::cout << "z[" << i << "] = " << z.transpose() << std::endl;
    // std::cout << "dual[" << i << "] = " << dual_[i].transpose() << std::endl;
    sol.weighted_position_.col(i) += rho * (gap_[i] + dual_[i]);
    sol.weights_[i] += rho;
  }

  iteration_ += 1;
  for (real& v: this->rho_) v *= 1.1;
  return sol;
}

template <idx dim> void Constraint_BallCollider<dim>::BeginStep() {
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

template <idx dim> real Constraint_BallCollider<dim>::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i : utils::iota(this->GetNumConstraints())) {
    math::vecr<dim> du = dual_[i] - fetch_from_global[i];
    gap_[i] += du;
    // std::cout << "gap[" << i << "] = " << gap_[i].transpose() << std::endl;
    sqr_prim_res += math::norm2(du);
  }
  return sqr_prim_res;
}

template <idx dim> void Constraint_BallCollider<dim>::EndStep() {}

template <idx dim> void Constraint_BallCollider<dim>::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

template <idx dim> void Constraint_BallCollider<dim>::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server<dim>();
  std::vector<idx> new_vertices;
  for (idx i : utils::iota(g.vertices_.cols())) {
    math::vecr<dim> const x = g.vertices_.col(i);
    if (math::norm(x - center_) < radius_ + tol_) {
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
      gap_.push_back(math::vecr<dim>::Zero());
      stiffness_.push_back(initial_rho_);
      this->rho_.push_back(initial_rho_);
    }
  }

  idx n_v = this->GetNumConstrainedVertices();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

template class Constraint_BallCollider<2>;
template class Constraint_BallCollider<3>;

}  // namespace ax::xpbd