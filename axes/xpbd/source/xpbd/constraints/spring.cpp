#include "ax/xpbd/constraints/spring.hpp"

#include "ax/core/config.hpp"
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

math::vecr<6> relax(const math::vecr<6>& y, const math::vecr<6>& z, real rho, real k, real L) {
  math::vecr<6> rho_z_minus_y = rho * (z - y);
  math::vecr<3> dRhs = rho_z_minus_y.template head<3>() - rho_z_minus_y.template tail<3>();
  real dx_norm = (2 * k * L + math::norm(dRhs)) / (2 * k + rho);

  // x1 - x2:
  math::vecr<3> dx = dRhs.normalized() * dx_norm;
  // x1 + x2:
  math::vecr<3> x_center = 0.5
                           * (z.template head<3>() + z.template tail<3>() - y.template head<3>()
                              - y.template tail<3>());
  math::vecr<6> x1x2;
  x1x2.template head<3>() = x_center + dx * 0.5;
  x1x2.template tail<3>() = x_center - dx * 0.5;
  return x1x2;
}

ConstraintSolution Constraint_Spring::SolveDistributed() {
  idx nV = this->GetNumConstrainedVertices();
  ConstraintSolution solution(nV);
  // Compute the relaxed solution:
  const auto& vert = this->constrained_vertices_position_;
  idx nC = this->GetNumConstraints();

  const auto& rho = this->rho_;
  real const rg2 = this->rho_global_ * this->rho_global_;
  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_[i];
    idx vi = ij[0], vj = ij[1];

    math::vecr<6> z;
    z.template head<3>() = vert[vi];
    z.template tail<3>() = vert[vj];
    math::vecr<6> y = gap_.col(i);
    real k = spring_stiffness_[i];
    real L = spring_length_[i];
    math::vecr<6> relaxed = relax(y, z, rho[i], k, L);
    math::vecr<6> old = dual_.col(i);
    dual_.col(i) = relaxed;
    solution.sqr_dual_residual_ += rg2 * math::norm2(old - relaxed);  // TODO: weighted by rho
    solution.weighted_position_.col(vi)
        += (relaxed.template head<3>() + y.template head<3>()) * rho[i];
    solution.weighted_position_.col(vj)
        += (relaxed.template tail<3>() + y.template tail<3>()) * rho[i];
    solution.weights_[vi] += rho[i];
    solution.weights_[vj] += rho[i];
  }

  return solution;
}

void Constraint_Spring::BeginStep() {
  idx nC = this->GetNumConstraints();
  auto const& g = ensure_server();
  this->UpdatePositionConsensus();
  dual_.resize(3 * 2, nC);
  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_[i];
    idx vi = ij.at(0), vj = ij.at(1);
    dual_.col(i).template head<3>() = g.vertices_.col(vi);
    dual_.col(i).template tail<3>() = g.vertices_.col(vj);
  }

  // this->rho_ = spring_stiffness_;
  this->rho_.resize(nC);
  for (idx i = 0; i < nC; ++i) {
    this->rho_[i] = spring_stiffness_[i];
  }
  this->rho_global_ = 1;
  gap_.setZero(3 * 2, nC);
}

real Constraint_Spring::UpdateDuality() {
  idx nC = this->GetNumConstraints();
  math::fieldr<3 * 2> prim_res = dual_;
  auto const& g = ensure_server();

  for (idx i = 0; i < nC; ++i) {
    auto const& ij = this->constraint_mapping_[i];
    idx vi_local = ij.at(0), vj_local = ij.at(1);
    idx vi = this->constrained_vertices_ids_[vi_local], vj = this->constrained_vertices_ids_[vj_local];
    auto const& xi = g.vertices_.col(vi);
    auto const& xj = g.vertices_.col(vj);
    prim_res.col(i).template head<3>() -= xi;
    prim_res.col(i).template tail<3>() -= xj;
  }

  gap_ += prim_res;
  return math::norm2(prim_res);
}

void Constraint_Spring::UpdateRhoConsensus(real scale) {
  // this->rho_ *= scale;
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  gap_ /= scale;
}
void Constraint_Spring::EndStep() {}

void Constraint_Spring::SetSprings(math::field2i const& indices, math::field1r const& stiffness) {
  this->constraint_mapping_ = indices;
  spring_stiffness_ = stiffness;
  spring_length_.resize(stiffness.size());
  std::set<idx> unique_vertices;

  auto const& g = ensure_server();
  for (idx i = 0; i < stiffness.size(); ++i) {
    auto const& ij = indices.col(i);
    spring_length_[i] = math::norm(g.vertices_.col(ij.x()) - g.vertices_.col(ij.y()));
    spring_stiffness_[i] /= spring_length_[i];

    unique_vertices.insert(ij.x());
    unique_vertices.insert(ij.y());
  }

  this->constrained_vertices_ids_.resize(unique_vertices.size());
  idx i = 0;
  std::map<idx, idx> inverse_map_;
  for (auto v : unique_vertices) {
    this->constrained_vertices_ids_[i] = v;
    inverse_map_[v] = i;
    i++;
  }

  for (idx i = 0; i < stiffness.size(); ++i) {
    auto const& ij = indices.col(i);
    idx vi = inverse_map_[ij.x()], vj = inverse_map_[ij.y()];
    this->constraint_mapping_[i].at(0) = vi;
    this->constraint_mapping_[i].at(1) = vj;
  }
}

}  // namespace ax::xpbd
