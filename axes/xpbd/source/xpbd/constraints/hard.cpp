#include "ax/xpbd/constraints/hard.hpp"

namespace ax::xpbd {
// f_i (x) = I(x)
// Augmented Lagrangian: f_i(x) + y_i.T x + rho/2 ||x - z_i||2
// Because f_i is the indicator function, the solution is x = x_hard.

template <idx dim> ConstraintSolution<dim> Constraint_Hard<dim>::SolveDistributed() {
  idx n_v = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> result(n_v);
  result.weights_ = this->rho_;
  for (idx i = 0; i < n_v; ++i) {
    result.weighted_position_.col(i) = (dual_.col(i) + gap_.col(i)) * this->rho_(i);
  }
  return result;
}

template <idx dim> void Constraint_Hard<dim>::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  gap_ += dual_ - fetch_from_global;
}

template <idx dim> void Constraint_Hard<dim>::BeginStep() {
  idx nV = this->GetNumConstrainedVertices();
  gap_.setZero(dim, nV);
  auto const& g = ensure_server<dim>();
  this->rho_.setConstant(nV, 1, initial_rho_ / (g.dt_ * g.dt_));
}

template <idx dim> void Constraint_Hard<dim>::EndStep() {}

template <idx dim>
void Constraint_Hard<dim>::SetHard(math::field1i const& indices,
                                   math::fieldr<dim> const& target_position) {
  this->constrained_vertices_ids_ = indices;
  this->constraint_mapping_ = indices;
  dual_ = target_position;
}

template <idx dim>
void Constraint_Hard<dim>::SetHard(math::field1i const& indices) {
  this->constrained_vertices_ids_ = indices;
  this->constraint_mapping_ = indices;
  dual_.setZero(dim, indices.size());
  this->UpdatePositionConsensus();
  for (idx i = 0; i < indices.size(); ++i) {
    dual_.col(i) = this->constrained_vertices_position_.col(i);
  }
}

template class Constraint_Hard<2>;
template class Constraint_Hard<3>;

}  // namespace ax::xpbd
