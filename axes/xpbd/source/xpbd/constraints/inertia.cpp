#include "ax/xpbd/constraints/inertia.hpp"

#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"
namespace ax::xpbd {

template <idx dim> ConstraintSolution<dim> Constraint_Inertia<dim>::SolveDistributed() {
  // Use M = mass / dt^2.
  // M/2 ||x_i - Y||^2 + rho/2 ||x_i - z_i^k + y_i||^2
  // Gradient = M(x_i - Y) + rho (x_i - z_i^k + y_i)
  // let ∂E = 0, and we got x_i =  (M Y + rho (z_i^k - y_i)) / (M+rho)
  idx nV = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> result(nV);
  const auto& rho = this->rho_;
  const auto& vert = this->constrained_vertices_position_;
  real dt = ensure_server<dim>().dt_, rg2 = this->rho_global_ * this->rho_global_;
  for (idx i : utils::iota(nV)) {
    real const mi = vertex_mass_[i] / (dt * dt);
    auto const& zi = vert[i];
    auto const& yi = gap_.col(i);
    auto const& Y = inertia_position_.col(i);
    math::vecr<dim> old = dual_.col(i);
    dual_.col(i) = (mi * Y + rho[i] * (zi - yi)) / (mi + rho[i]);
    result.sqr_dual_residual_ += rg2 * math::norm2(old - dual_.col(i)); // TODO: weighted by rho
  }
  
  for (idx i: utils::iota(nV)) {
    auto const& yi = gap_.col(i);
    auto const& X = dual_.col(i);
    result.weighted_position_.col(i) = (X + yi) * rho[i];
    result.weights_[i] = rho[i];
  }
  return result;
}

template <idx dim> real Constraint_Inertia<dim>::UpdateDuality() {
  real prim_res = 0;
  for (idx i: utils::iota(this->GetNumConstrainedVertices())) {
    math::vecr<dim> residual = dual_.col(i) - this->constrained_vertices_position_[i];
    gap_.col(i) += residual;
    prim_res += residual.squaredNorm();
  }
  return prim_res;
}

template<idx dim> void Constraint_Inertia<dim>::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->gap_ /= scale;
  this->rho_global_ *= scale;
}

template <idx dim> void Constraint_Inertia<dim>::BeginStep() {
  auto& g = ensure_server<dim>();
  idx nV = g.vertices_.cols();
  real const dt = g.dt_;

  this->constrained_vertices_ids_.resize(nV);
  this->rho_.resize(nV);
  for (idx i : utils::iota(nV)) {
    this->rho_[i] = g.mass_[i] / (dt * dt);
    this->constrained_vertices_ids_[i] = i;
  }
  this->UpdatePositionConsensus();

  vertex_mass_ = g.mass_;
  dual_ = g.vertices_;
  gap_.setZero(dim, nV);
  this->rho_global_ = 1;
  inertia_position_ = g.vertices_;
}

template <idx dim> void Constraint_Inertia<dim>::EndStep() {}

template class Constraint_Inertia<2>;
template class Constraint_Inertia<3>;

}  // namespace ax::xpbd
