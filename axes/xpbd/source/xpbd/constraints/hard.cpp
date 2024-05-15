#include "ax/xpbd/constraints/hard.hpp"

namespace ax::xpbd {

template <idx dim> ConstraintSolution<dim> Constraint_Hard<dim>::SolveConsensus() {
  idx n_v = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> result(n_v);
  result.weights_.setOnes();
  result.weighted_position_.noalias() = dual_ + gap_ / this->rho_;
  return result;
}

template <idx dim> void Constraint_Hard<dim>::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  gap_.noalias() += this->rho_ * (dual_ - fetch_from_global);
}

template <idx dim> void Constraint_Hard<dim>::BeginStep() {
  idx nV = this->GetNumConstrainedVertices();
  dual_.setZero(dim, nV);
  gap_.setZero(dim, nV);
}

template <idx dim> void Constraint_Hard<dim>::EndStep() {}

template class Constraint_Hard<2>;
template class Constraint_Hard<3>;

}  // namespace ax::xpbd