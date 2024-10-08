#include "ax/xpbd/constraints/hard.hpp"
#include "ax/math/linalg.hpp"

namespace ax::xpbd {
// f_i (x) = I(x)
// Augmented Lagrangian: f_i(x) + y_i.T x + rho/2 ||x - z_i||2
// Because f_i is the indicator function, the solution is x = x_hard.

ConstraintSolution Constraint_Hard::SolveDistributed() {
  Index n_v = this->GetNumConstrainedVertices();
  ConstraintSolution result(n_v);
  for (Index i = 0; i < n_v; ++i) {
    result.weighted_position_.col(i) = (dual_.col(i) + gap_.col(i)) * this->rho_[i];
    result.weights_[i] = this->rho_[i];
  }
  result.sqr_dual_residual_ = 0;
  return result;
}

Real Constraint_Hard::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  // auto const prim_res = dual_ - fetch_from_global;
  // gap_ += dual_ - fetch_from_global;
  Real prim_res = 0;
  for (Index i = 0; i < this->GetNumConstrainedVertices(); ++i) {
    math::RealVector3 residual = dual_.col(i) - fetch_from_global[i];
    gap_.col(i) += residual;

    prim_res += residual.squaredNorm();
  }
  return prim_res;
}

void Constraint_Hard::BeginStep() {
  Index nV = this->GetNumConstrainedVertices();
  gap_.setZero(3, nV);
  auto const& g = ensure_server();
  this->rho_.resize(nV, initial_rho_);
  this->rho_global_ = 1.;
}

void Constraint_Hard::EndStep() {}

void Constraint_Hard::SetHard(math::IndexField1 const& indices, math::RealField3 const& target_position) {
  this->constraint_mapping_ = indices;
  this->constrained_vertices_ids_ = this->constraint_mapping_.Mapping();
  dual_ = target_position;
}

void Constraint_Hard::SetHard(math::IndexField1 const& indices) {
  this->constraint_mapping_ = indices;
  this->constrained_vertices_ids_ = this->constraint_mapping_.Mapping();
  dual_.setZero(3, indices.size());

  this->UpdatePositionConsensus();
  for (Index i = 0; i < indices.size(); ++i) {
    dual_.col(i) = this->constrained_vertices_position_[i];
  }
}

}  // namespace ax::xpbd
