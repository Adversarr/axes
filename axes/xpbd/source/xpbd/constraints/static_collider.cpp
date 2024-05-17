#include "ax/xpbd/constraints/static_collider.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

template <idx dim>
ConstraintSolution<dim> Constraint_StaticCollider<dim>::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  idx const nC = this->GetNumConstraints();
  idx const nV = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> sol(nV);

  for (auto i: utils::iota(nC)) {
    real rho = this->rho_[i];
    idx const vi = this->constraint_mapping_(0, i);
    math::vecr<dim> const& x = this->constrained_vertices_position_.col(i);
    dual_[i] = test_fn_(x);
    sol.weighted_position_.col(vi) += rho * (gap_[i] + dual_[i]);
    sol.weights_(vi) += rho;
  }
  return sol;
}

template <idx dim>
void Constraint_StaticCollider<dim>::BeginStep() {
  gap_.clear();
  dual_.clear();
  collidings_.clear();
}

template <idx dim>
real Constraint_StaticCollider<dim>::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i: utils::iota(this->GetNumConstraints())) {
    math::vecr<dim> du = dual_[i] - fetch_from_global.col(i);
    gap_[i] += du;
    sqr_prim_res += math::norm2(du);
  }
  return sqr_prim_res;
}

template <idx dim>
void Constraint_StaticCollider<dim>::EndStep() {}

template <idx dim>
void Constraint_StaticCollider<dim>::UpdateRhoConsensus(real scale) {
  this->rho_ *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

template <idx dim>
void Constraint_StaticCollider<dim>::UpdatePositionConsensus() {
  idx n_v = this->GetNumConstrainedVertices();
  auto const& cmap = this->constrained_vertices_ids_;
  math::fieldr<dim>& local = this->constrained_vertices_position_;
  local.resize(dim, n_v);
  auto const& g = ensure_server<dim>();
  std::vector<idx> new_vertices;
  for (idx i : utils::iota(g.vertices_.cols())) {
    math::vecr<dim> const x = g.vertices_.col(i);
    math::vecr<dim> const p = test_fn_(x);
    if (p != x) {
      if (collidings_.find(i) == collidings_.end()) {
        collidings_.insert(i);
        new_vertices.push_back(i);
      }
    }
  }

  if (new_vertices.size() > 0) {
    dual_.resize(dual_.size() + new_vertices.size());
    gap_.resize(gap_.size() + new_vertices.size());
    this->constrained_vertices_position_.resize(dim, n_v + new_vertices.size());
    this->constraint_mapping_.resize(1, n_v + new_vertices.size());
    this->constrained_vertices_ids_.resize(n_v + new_vertices.size());
    for (idx i : utils::iota(new_vertices.size())) {
      idx iV = new_vertices[i];
      this->constrained_vertices_ids_[n_v + i] = iV;
      this->constraint_mapping_(0, n_v + i) = iV;
      this->constrained_vertices_position_.col(n_v + i) = g.vertices_.col(iV);
      dual_[n_v + i] = test_fn_(g.vertices_.col(iV));
      gap_[n_v + i] = dual_[n_v + i] - g.vertices_.col(iV);
    }
    this->rho_.resize(n_v + new_vertices.size());
    this->rho_.setConstant(initial_rho_ * this->rho_global_);
  }

  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local.col(i) = g.vertices_.col(iV);
  }
}

template class Constraint_StaticCollider<2>;
template class Constraint_StaticCollider<3>;

}