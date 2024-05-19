#include "ax/xpbd/constraints/hard.hpp"
#include "ax/math/linalg.hpp"

namespace ax::xpbd {
// f_i (x) = I(x)
// Augmented Lagrangian: f_i(x) + y_i.T x + rho/2 ||x - z_i||2
// Because f_i is the indicator function, the solution is x = x_hard.

template <idx dim> ConstraintSolution<dim> Constraint_Hard<dim>::SolveDistributed() {
  idx n_v = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> result(n_v);
  for (idx i = 0; i < n_v; ++i) {
    result.weighted_position_.col(i) = (dual_.col(i) + gap_.col(i)) * this->rho_[i];
    result.weights_[i] = this->rho_[i];
  }
  result.sqr_dual_residual_ = 0;
  return result;
}

template <idx dim> real Constraint_Hard<dim>::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  // auto const prim_res = dual_ - fetch_from_global;
  // gap_ += dual_ - fetch_from_global;
  real prim_res = 0;
  for (idx i = 0; i < this->GetNumConstrainedVertices(); ++i) {
    math::vecr<dim> residual = dual_.col(i) - fetch_from_global[i];
    gap_.col(i) += residual;

    prim_res += residual.squaredNorm();
  }
  return prim_res;
}

template <idx dim> void Constraint_Hard<dim>::BeginStep() {
  idx nV = this->GetNumConstrainedVertices();
  gap_.setZero(dim, nV);
  auto const& g = ensure_server<dim>();
  this->rho_.resize(nV, initial_rho_ / (g.dt_ * g.dt_));
  this->rho_global_ = 1.;
}

template <idx dim> void Constraint_Hard<dim>::EndStep() {}

template <idx dim>
void Constraint_Hard<dim>::SetHard(math::field1i const& indices,
                                   math::fieldr<dim> const& target_position) {
  this->constraint_mapping_ = indices;
  this->constrained_vertices_ids_ = this->constraint_mapping_.Mapping();
  dual_ = target_position;
}

template <idx dim>
void Constraint_Hard<dim>::SetHard(math::field1i const& indices) {
  this->constraint_mapping_ = indices;
  this->constrained_vertices_ids_ = this->constraint_mapping_.Mapping();
  dual_.setZero(dim, indices.size());

  this->UpdatePositionConsensus();
  for (idx i = 0; i < indices.size(); ++i) {
    dual_.col(i) = this->constrained_vertices_position_[i];
  }
}

template class Constraint_Hard<2>;
template class Constraint_Hard<3>;

}  // namespace ax::xpbd
