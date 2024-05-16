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
    auto const& zi = vert.col(i);
    auto const& yi = gap_.col(i);
    auto const& Y = inertia_position_.col(i);
    math::vecr<dim> old = dual_.col(i);
    dual_.col(i) = (mi * Y + rho[i] * (zi - yi)) / (mi + rho[i]);
    result.sqr_dual_residual_ += rg2 * math::norm2(old - dual_.col(i)); // TODO: weighted by rho
  }
  
  result.weights_ = rho;
  for (idx i: utils::iota(nV)) {
    auto const& yi = gap_.col(i);
    auto const& X = dual_.col(i);
    result.weighted_position_.col(i) = (X + yi) * rho[i];
  }
  return result;
}

template <idx dim> real Constraint_Inertia<dim>::UpdateDuality() {
  math::fieldr<dim> prim_res = dual_ - this->constrained_vertices_position_;
  gap_.noalias() += prim_res;
  return prim_res.squaredNorm();
  // update rho: TODO: Not work.
  // idx nV = this->GetNumConstrainedVertices();
  // auto& rho = this->rho_;
  // real pdt = this->primal_dual_threshold_, dpt = this->dual_primal_threshold_,
  //      pdr = this->primal_dual_ratio_, dpr = this->dual_primal_ratio_;
  // for (idx i: utils::iota(nV)) {
  //   // change of dual
  //   real dual_residual = rho[i] * math::norm(dual_.col(i) - dual_old_.col(i));
  //   real prim_residual = math::norm(prim_res.col(i));
  //   if (prim_residual / dual_residual > pdt) {
  //     rho[i] *= pdr;
  //     gap_.col(i) /= pdr;
  //   } else if (dual_residual / prim_residual > dpt) {
  //     rho[i] /= dpr;
  //     gap_.col(i) *= dpr;
  //   }
  // }
}

template<idx dim> void Constraint_Inertia<dim>::UpdateRhoConsensus(real scale) {
  this->rho_ *= scale;
  this->gap_ /= scale;
  this->rho_global_ *= scale;
}

template <idx dim> void Constraint_Inertia<dim>::BeginStep() {
  auto& g = ensure_server<dim>();
  idx nV = g.vertices_.cols();
  this->constrained_vertices_ids_.resize(1, nV);
  for (idx i : utils::iota(nV)) {
    this->constrained_vertices_ids_(0, i) = i;
  }
  this->constraint_mapping_ = this->constrained_vertices_ids_;
  this->UpdatePositionConsensus();

  vertex_mass_ = g.mass_;
  dual_ = g.vertices_;
  gap_.setZero(dim, nV);
  real const dt = g.dt_;
  this->rho_ = g.mass_ / (dt * dt);
  this->rho_global_ = 1;
  inertia_position_ = g.vertices_;
}

template <idx dim> void Constraint_Inertia<dim>::EndStep() {}

template class Constraint_Inertia<2>;
template class Constraint_Inertia<3>;

}  // namespace ax::xpbd
