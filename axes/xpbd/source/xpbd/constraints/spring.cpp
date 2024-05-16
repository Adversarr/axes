#include "ax/xpbd/constraints/spring.hpp"

#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"
namespace ax::xpbd {

// NOTE:
//
// Solves the consensus problem for the spring constraint.
//
// The spring energy:
//   f(x) = 1/2 k (|| Dx || - L)^2
// with Augmented Lagrangian:
//   f(x) + 1/2 rho || x - z + y ||^2
// Take derivative:
//   Df(x) + y + rho (x - z) = 0 => Df(x) + rho x = -rho y + rho z
//
// Derivation:
//   D.t k(|Dx| - L) Dx/|Dx| +   rho x =   rho(z - y)
// Apply D on the left:
// =>  2 k(|Dx| - L) Dx/|Dx| + rho D x = D rho(z - y)
// Take norm:
// =>  2 k(|Dx| - L) + rho |Dx| = || D rho(z - y) ||
//  |Dx| = (2 k L + || D rho(z - y) ||) / (2 k + rho)
// and Dx / |Dx| = (z - y) / || z - y ||
// This can give us Dx.
// What about Apply [I; I] to it:
// rho (x1 + x2) = rho (z1 + z2) - rho (y1 + y2)
// => x1 + x2 = z1 + z2 - y1 - y2
// Apply D again:

template <idx dim> math::vecr<dim * 2> relax(const math::vecr<dim * 2>& y,
                                             const math::vecr<dim * 2>& z, real rho, real k,
                                             real L) {
  math::vecr<dim * 2> rho_z_minus_y = rho * (z - y);
  math::vecr<dim> dRhs = rho_z_minus_y.template head<dim>() - rho_z_minus_y.template tail<dim>();
  real dx_norm = (2 * k * L + math::norm(dRhs)) / (2 * k + rho);

  // x1 - x2:
  math::vecr<dim> dx = dRhs.normalized() * dx_norm;
  // x1 + x2:
  math::vecr<dim> x_center = 0.5 * (z.template head<dim>() + z.template tail<dim>()
                                    - y.template head<dim>() - y.template tail<dim>());
  math::vecr<dim * 2> x1x2;
  x1x2.template head<dim>() = x_center + dx * 0.5;
  x1x2.template tail<dim>() = x_center - dx * 0.5;
  return x1x2;
}

template <idx dim> ConstraintSolution<dim> Constraint_Spring<dim>::SolveDistributed() {
  idx nV = this->GetNumConstrainedVertices();
  ConstraintSolution<dim> solution(nV);
  dual_old_ = dual_;

  // Compute the relaxed solution:
  const auto& vert = this->constrained_vertices_position_;
  idx nC = this->GetNumConstraints();

  const auto& rho = this->rho_;
  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_.col(i);
    idx vi = ij.x(), vj = ij.y();

    math::vecr<dim * 2> z;
    z.template head<dim>() = vert.col(vi);
    z.template tail<dim>() = vert.col(vj);
    math::vecr<dim * 2> y = gap_.col(i);
    real k = spring_stiffness_[i];
    real L = spring_length_[i];
    math::vecr<dim * 2> relaxed = relax<dim>(y, z, rho[i], k, L);
    dual_.col(i) = relaxed;
    solution.weighted_position_.col(vi) += (relaxed.template head<dim>() + y.template head<dim>()) * rho[i];
    solution.weighted_position_.col(vj) += (relaxed.template tail<dim>() + y.template tail<dim>()) * rho[i];
    solution.weights_[vi] += rho[i];
    solution.weights_[vj] += rho[i];
  }

  return solution;
}

template <idx dim> void Constraint_Spring<dim>::BeginStep() {
  idx nC = this->GetNumConstraints();
  auto const& g = ensure_server<dim>();
  this->UpdatePositionConsensus();
  dual_.resize(dim * 2, nC);
  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_.col(i);
    idx vi = ij.x(), vj = ij.y();
    dual_.col(i).template head<dim>() = g.vertices_.col(vi);
    dual_.col(i).template tail<dim>() = g.vertices_.col(vj);
  }

  this->rho_ = spring_stiffness_;
  gap_.setZero(dim * 2, nC);
}

template <idx dim> void Constraint_Spring<dim>::UpdateDuality() {
  idx nC = this->GetNumConstraints();
  math::fieldr<dim * 2> prim_res = dual_;
  auto const& g = ensure_server<dim>();

  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_.col(i);
    idx vi_local = ij.x(), vj_local = ij.y();
    idx vi = this->constrained_vertices_ids_[vi_local], vj = this->constrained_vertices_ids_[vj_local];
    auto const& xi = g.vertices_.col(vi);
    auto const& xj = g.vertices_.col(vj);
    prim_res.col(i).template head<dim>() -= xi;
    prim_res.col(i).template tail<dim>() -= xj;
  }

  // std::cout << prim_res << std::endl;
  gap_ += prim_res;

  // update rho: TODO: Not work.
  // idx nV = this->GetNumConstraints();
  // auto& rho = this->rho_;
  // real pdt = this->primal_dual_threshold_, dpt = this->dual_primal_threshold_,
  //      pdr = this->primal_dual_ratio_, dpr = this->dual_primal_ratio_;
  // for (idx i: utils::iota(nV)) {
  //   // change of dual
  //   real prim_residual = math::norm(prim_res.col(i));
  //   real dual_residual = rho[i] * math::norm(dual_.col(i) - dual_old_.col(i));
  //   if (prim_residual / dual_residual > pdt) {
  //     rho[i] *= pdr;
  //     gap_.col(i) /= pdr;
  //     AX_LOG(INFO) << "Enlarge: " << i << " ==> " << prim_residual << ":" << dual_residual;
  //   } else if (dual_residual / prim_residual > dpt) {
  //     rho[i] /= dpr;
  //     gap_.col(i) *= dpr;
  //     AX_LOG(INFO) << "Shrink: " << i << " ==> " << prim_residual << ":" << dual_residual;
  //   }
  // }
}

template <idx dim> void Constraint_Spring<dim>::EndStep() {}

template <idx dim> void Constraint_Spring<dim>::SetSprings(math::field2i const& indices,
                                                           math::field1r const& stiffness) {
  this->constraint_mapping_ = indices;
  spring_stiffness_ = stiffness;
  spring_length_.resize(stiffness.size());
  std::set<idx> unique_vertices; // BUG: assume every vertex in scene has this constraint.

  auto const& g = ensure_server<dim>();
  for (idx i = 0; i < stiffness.size(); ++i) {
    auto const& ij = indices.col(i);
    spring_length_[i] = math::norm(g.vertices_.col(ij.x()) - g.vertices_.col(ij.y()));
    spring_stiffness_[i] /= spring_length_[i];

    unique_vertices.insert(ij.x());
    unique_vertices.insert(ij.y());
  }

  this->constrained_vertices_ids_.resize(1, unique_vertices.size());
  idx i = 0;
  std::map<idx, idx> inverse_map_;
  for (auto v : unique_vertices) {
    this->constrained_vertices_ids_(0, i) = v;
    inverse_map_[v] = i;
    i++;
  }

  for (idx i = 0; i < stiffness.size(); ++i) {
    auto const& ij = indices.col(i);
    idx vi = inverse_map_[ij.x()], vj = inverse_map_[ij.y()];
    this->constraint_mapping_.col(i) = math::vec2i{vi, vj};
  }
}

template class Constraint_Spring<2>;
template class Constraint_Spring<3>;

}  // namespace ax::xpbd
