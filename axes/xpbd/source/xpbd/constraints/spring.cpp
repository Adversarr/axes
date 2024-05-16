#pragma once
#include "ax/xpbd/constraints/spring.hpp"

#include "ax/math/linalg.hpp"
namespace ax::xpbd {

// NOTE:
//
// Solves the consensus problem for the spring constraint.
//
// The spring energy:
//   f(x) = 1/2 k (|| Dx || - L)^2
// with Augmented Lagrangian:
//   f(x) + y.T x + 1/2 rho || x - z ||^2
// Take derivative:
//   Df(x) + y + rho (x - z) = 0 => Df(x) + rho x = -y + rho z
//
// Derivation:
//   D.t k(|Dx| - L) Dx/|Dx| = -y + rho z
//
// Apply D again:

template <idx dim> math::vecr<dim * 2> relax(const math::vecr<dim * 2>& y,
                                             const math::vecr<dim * 2>& z, real rho, real k,
                                             real L) {
  math::vecr<dim * 2> rho_z_minus_y = rho * z - y;
  math::vecr<dim> dRhs = rho_z_minus_y.template head<dim>() - rho_z_minus_y.template tail<dim>();
  real dx_norm = (2 * k * L + math::norm(dRhs)) / (2 * k + rho);

  // x1 - x2:
  math::vecr<dim> dx = dRhs.normalized() * dx_norm;
  // x1 + x2:
  math::vecr<dim> x_center = (z.template head<dim>() + z.template tail<dim>()) * 0.5
                             - 0.5 / rho * (y.template head<dim>() + y.template tail<dim>());
  math::vecr<dim * 2> x1x2;
  x1x2.template head<dim>() = x_center + dx * 0.5;
  x1x2.template tail<dim>() = x_center - dx * 0.5;
}

template <idx dim> ConstraintSolution<dim> Constraint_Spring<dim>::SolveDistributed() {
  idx nV = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> solution(nV);

  // Compute the relaxed solution:
  const auto& vert = this->constrained_vertices_position_;
  idx nC = this->GetNumConstriants();

  real rho = this->rho_;
  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_.col(i);
    idx vi = ij.x(), vj = ij.y();

    math::vecr<dim * 2> z;
    z.template head<dim>() = vert.col(vi);
    z.template tail<dim>() = vert.col(vj);
    math::vecr<dim * 2> y = gap_.col(i);
    real k = spring_stiffness_[i];
    real L = spring_length_[i];
    math::vecr<dim * 2> relaxed = relax(y, z, rho, k, L);

    dual_.col(i) = relaxed;
    solution.weighted_position_.col(vi) += relaxed.template head<dim>();
    solution.weighted_position_.col(vj) += relaxed.template tail<dim>();
    solution.weights_[vi] += 1;
    solution.weights_[vj] += 1;
  }

  return solution;
}

template <idx dim> void Constraint_Spring<dim>::BeginStep() {
  idx nC = this->GetNumConstraints();
  auto const& g = ensure_server<dim>();
  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_.col(i);
    idx vi = ij.x(), vj = ij.y();
    dual_.col(i).template head<dim>() = g.vertices_.col(vi);
    dual_.col(i).template tail<dim>() = g.vertices_.col(vj);
  }

  gap_.setZero();
}

template <idx dim> void Constraint_Spring<dim>::UpdateDuality() {
  idx nC = this->GetNumConstraints();
  auto const& g = ensure_server<dim>();
  real const rho = this->rho_;

  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_.col(i);
    idx vi = ij.x(), vj = ij.y();
    auto const& xi = g.vertices_.col(vi);
    auto const& xj = g.vertices_.col(vj);

    gap_.col(i).template head<dim>() += rho * xi;
    gap_.col(i).template tail<dim>() += rho * xj;
    gap_.col(i) -= rho * dual_.col(i);
  }
}

template <idx dim> void Constraint_Spring<dim>::EndStep() {}

template <idx dim> void Constraint_Spring<dim>::SetSprings(math::field2i const& indices,
                                                           math::field1r const& stiffness) {
  this->constraint_mapping_ = indices;
  spring_stiffness_ = stiffness;
  spring_length_.resize(stiffness.size());

  auto const& g = ensure_server<dim>();
  for (idx i = 0; i < stiffness.size(); ++i) {
    spring_length_[i] = 0;
    auto const& ij = indices.col(i);
    spring_length_[i] = math::norm(g.vertices_.col(ij.x()) - g.vertices_.col(ij.y()));
  }
}

}  // namespace ax::xpbd
