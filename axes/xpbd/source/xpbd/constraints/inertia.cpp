#include "ax/xpbd/constraints/inertia.hpp"

#include "ax/utils/iota.hpp"
namespace ax::xpbd {

template <idx dim> ConstraintSolution<dim> Constraint_Inertia<dim>::SolveConsensus() {
  // M/2 ||x_i - Y||^2 + y_i.T x + rho/2 ||x_i - z_i^k||^2
  // Gradient = M(x_i - Y) + rho (x_i - z_i^k) + y_i.T
  // let ∂E = 0, and we got x_i =  (M Y + rho z_i^k - y_i.T) / (M+rho)
  idx nV = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> result(nV);
  result.weights_.setOnes();
  const real rho = this->rho_;
  const auto& vert = this->constrained_vertices_position_;
  for (idx i : utils::iota(nV)) {
    real const mi = vertex_mass_[i];
    auto const& zi = vert.col(i);
    auto const& yi = gap_.col(i);
    auto const& Y = inertia_position_.col(i);
    result.weighted_position_.col(i) = (mi * Y + rho * zi - yi) / (mi + rho);
  }
  return result;
}

template <idx dim> void Constraint_Inertia<dim>::BeginStep() {
  idx nV = this->GetNumConstrainedVertices();
  dual_.setZero(dim, nV);
  gap_.setZero(dim, nV);
  // TODO: Setup Inertia Positions.
}

template <idx dim> void Constraint_Inertia<dim>::EndStep() {}

}  // namespace ax::xpbd