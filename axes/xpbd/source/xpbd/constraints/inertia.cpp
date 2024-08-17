#include "ax/xpbd/constraints/inertia.hpp"

#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"
namespace ax::xpbd {

ConstraintSolution Constraint_Inertia::SolveDistributed() {
  // Use M = mass / dt^2.
  // M/2 ||x_i - Y||^2 + rho/2 ||x_i - z_i^k + y_i||^2
  // Gradient = M(x_i - Y) + rho (x_i - z_i^k + y_i)
  // let ∂E = 0, and we got x_i =  (M Y + rho (z_i^k - y_i)) / (M+rho)
  Index nV = this->GetNumConstrainedVertices();
  ConstraintSolution result(nV);
  const auto& rho = this->rho_;
  const auto& vert = this->constrained_vertices_position_;
  real dt = ensure_server().dt_, rg2 = this->rho_global_ * this->rho_global_;
  for (Index i : utils::iota(nV)) {
    real const mi = vertex_mass_[i];
    auto const& zi = vert[i];
    auto const& yi = gap_.col(i);
    auto const& Y = inertia_position_.col(i);
    math::RealVector3 old = dual_.col(i);
    dual_.col(i) = (mi * Y + rho[i] * (zi - yi)) / (mi + rho[i]);
    result.sqr_dual_residual_ += rg2 * math::norm2(old - dual_.col(i)); // TODO: weighted by rho
  }
  
  for (Index i: utils::iota(nV)) {
    auto const& yi = gap_.col(i);
    auto const& X = dual_.col(i);
    result.weighted_position_.col(i) = (X + yi) * rho[i];
    result.weights_[i] = rho[i];
  }
  return result;
}

real Constraint_Inertia::UpdateDuality() {
  real prim_res = 0;
  for (Index i: utils::iota(this->GetNumConstrainedVertices())) {
    math::RealVector3 residual = dual_.col(i) - this->constrained_vertices_position_[i];
    gap_.col(i) += residual;
    prim_res += residual.squaredNorm();
  }
  return prim_res;
}

void Constraint_Inertia::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->gap_ /= scale;
  this->rho_global_ *= scale;
}

void Constraint_Inertia::BeginStep() {
  auto& g = ensure_server();
  Index nV = g.vertices_.cols();

  this->constrained_vertices_ids_.resize(nV);
  this->rho_.resize(nV);
  for (Index i : utils::iota(nV)) {
    this->rho_[i] = g.mass_[i];
    this->constrained_vertices_ids_[i] = i;
  }
  this->UpdatePositionConsensus();

  vertex_mass_ = g.mass_;
  dual_ = g.vertices_;
  gap_.setZero(3, nV);
  this->rho_global_ = 1;
  inertia_position_ = g.vertices_;
}

void Constraint_Inertia::EndStep() {}

}  // namespace ax::xpbd
