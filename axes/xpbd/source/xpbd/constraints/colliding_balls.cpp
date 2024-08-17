#include "ax/xpbd/constraints/colliding_balls.hpp"

#include "ax/utils/iota.hpp"
#include "ax/xpbd/details/relaxations.hpp"

namespace ax::xpbd {

ConstraintSolution Constraint_CollidingBalls::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  Index const nC = this->GetNumConstraints();
  Index const nV = this->GetNumConstrainedVertices();
  // std::cout << "SolveDistributed: " << nC << " " << nV << std::endl;
  ConstraintSolution sol(nV);

  auto dual_old = dual_;
  for (auto i : utils::iota(nC)) {
    real rho = this->rho_[i];
    auto C = constraint_mapping_[i];
    m32 z;
    z.col(0) = constrained_vertices_position_[C[0]];
    z.col(1) = constrained_vertices_position_[C[1]];
    auto const& u = gap_[i];
    auto& d = dual_[i];
    auto& k = stiffness_[i];
    relax_vertex_vertex_impl(z, u, origin_[i], d, k, rho, ball_radius_, tol_);
    m32 const rhogd = (u + d) * rho;
    auto const& dold = dual_old[i];
    sol.sqr_dual_residual_ += (d - dold).squaredNorm();
    ConstraintMap::ConstVisitor v = constraint_mapping_[i];
    for (Index i = 0; i < 2; ++i) {
      sol.weighted_position_.col(v[i]) += rhogd.col(i);
      sol.weights_[v[i]] += rho;
    }
  }
  iteration_ += 1;
  return sol;
}

void Constraint_CollidingBalls::BeginStep() {
  // Initialize the dual variable.
  dual_.clear();
  gap_.clear();
  origin_.clear();
  colliding_map_.clear();
  colliding_vertices_.clear();
  collidings_.clear();
  global_to_local_.clear();
  iteration_ = 0;
  rho_.clear();
  stiffness_.clear();
  constrained_vertices_ids_.clear();
  constrained_vertices_position_.clear();
  constraint_mapping_.clear();
}

real Constraint_CollidingBalls::UpdateDuality() {
  real sqr_dual_residual = 0;
  auto& local = this->constrained_vertices_position_;
  for (auto i : utils::iota(GetNumConstraints())) {
    auto& g = gap_[i];
    auto const& d = dual_[i];
    m32 z;
    auto C = constraint_mapping_[i];
    z.col(0) = local[C[0]];
    z.col(1) = local[C[1]];
    g += d - z;
    sqr_dual_residual += (d - z).squaredNorm();
  }
  return sqr_dual_residual;
}

void Constraint_CollidingBalls::EndStep() {}

void Constraint_CollidingBalls::UpdateRhoConsensus(real scale) {
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_CollidingBalls::UpdatePositionConsensus() {
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<std::pair<Index, Index>> new_collisions;
  std::vector<Index> new_colliding_vertices;

  auto put_vert = [&](Index v) {
    if (colliding_vertices_.insert(v).second) {
      new_colliding_vertices.push_back(v);
    }
  };

  auto put_coll = [this](std::pair<Index, Index> vv) -> bool {
    auto it = collidings_.find(vv);
    if (it == collidings_.end()) {
      collidings_.emplace(vv, 1);
      return true;
    } else {
      it->second += 1;
      return false;
    }
  };

  // Current implementation is brute force.
  Index const nV = g.vertices_.cols();
  for (Index i : utils::iota(nV)) {
    for (Index j = i + 1; j < nV; ++j) {
      v3 const& pi = g.vertices_.col(i);
      v3 const& pj = g.vertices_.col(j);
      real distance = math::norm(pi - pj);
      if (distance < ball_radius_ + tol_ * 3) {
        // std::cout << "collision: " << i << " " << j << " d: " << distance << std::endl;
        if (put_coll({i, j})) {
          put_vert(i);
          put_vert(j);
          new_collisions.push_back({i, j});
        }
      }
    }
  }

  real const dt2 = g.dt_ * g.dt_;
  if (new_collisions.size() > 0) {
    for (auto v : new_colliding_vertices) {
      constrained_vertices_ids_.push_back(v);
      global_to_local_.emplace(v, constrained_vertices_ids_.size() - 1);
      constrained_vertices_position_.push_back(g.vertices_.col(v));
    }

    for (auto [i, j] : new_collisions) {
      Index const vi = global_to_local_.at(i);
      Index const vj = global_to_local_.at(j);
      constraint_mapping_.emplace_back(vi, vj);
      auto& dual = dual_.emplace_back();
      auto& gap = gap_.emplace_back();
      dual.col(0) = g.last_vertices_.col(i);
      dual.col(1) = g.last_vertices_.col(j);
      gap.col(0) = g.vertices_.col(i);
      gap.col(1) = g.vertices_.col(j);
      gap = (dual - gap);
      rho_.push_back(initial_rho_ * dt2);
      stiffness_.push_back(initial_rho_ * dt2);

      origin_.push_back(dual);
      colliding_map_.emplace(std::minmax(i, j), rho_.size() - 1);
    }
  }

  // now update the position.
  Index nCV = GetNumConstrainedVertices();
  for (Index i = 0; i < nCV; ++i) {
    Index const v = constrained_vertices_ids_[i];
    local[i] = g.vertices_.col(v);
  }
}

}  // namespace ax::xpbd
