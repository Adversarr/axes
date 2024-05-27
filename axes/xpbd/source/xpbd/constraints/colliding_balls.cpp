#include "ax/xpbd/constraints/colliding_balls.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

using m32 = math::matr<3, 2>;
using v3 = math::vec3r;
static void relax(real rho, real& k, m32 const& z, m32 const& u, m32& dual, real radius, real eps) {
  // Very similar to spring, let L = radius + eps.
  // The spring energy:
  //   f(x) = 1/2 k (|| D x || - (L + eps))^2
  // with Augmented Lagrangian:
  //   f(x) + 1/2 rho || x - z + u ||^2
  // Take derivative:
  //   D.T k (|Dx| - (L + eps)) (Dx)/|Dx| + rho (x - z + u) = 0
  // Apply D again, and take norm.
  //   k(|Dx| - (L + eps)) + rho |Dx| = rho |Dz - Du|
  // => |Dx| = (rho |Dz - Du| + k (L + eps)) / (k + rho)
  // we expect |Dx| >= L.
  //   k (L + eps) + rho |Dz - Du| >= (k + rho) L
  //   k eps >= rho (L - |Dz - Du|)
  // if |Dz - Du| > L, nothing will be relaxed.
  m32 const zu = z - u; // (z - u)
  v3 const c = 0.5 * (zu.col(0) + zu.col(1));
  v3 const d = zu.col(0) - zu.col(1);

  real const d_norm = d.norm();
  if (d_norm > radius) {
    dual = zu;
    return;
  }

  real dx_norm = (rho * d_norm + k * (radius + eps)) / (k + rho);
  if (dx_norm < radius) {
    k = 4 * rho * (radius - d_norm) / eps;
    dx_norm = (rho * d_norm + k * (radius + eps)) / (k + rho);
  }

  v3 const dn = math::normalized(d);
  dual.col(0) = c + dx_norm * dn;
  dual.col(1) = c - dx_norm * dn;
}

ConstraintSolution Constraint_CollidingBalls::SolveDistributed() {
  // For each collition, project to the closest point on the surface.
  idx const nC = this->GetNumConstraints();
  idx const nV = this->GetNumConstrainedVertices();
  ConstraintSolution sol(nV);

  auto dual_old = dual_;
  for (auto i : utils::iota(nC)) {
    real& rho = this->rho_[i];
    auto C = constraint_mapping_[i];
    m32 z;
    z.col(0) = constrained_vertices_position_[C[0]];
    z.col(1) = constrained_vertices_position_[C[1]];
    auto& g = gap_[i];
    auto& d = dual_[i];
    relax(rho, stiffness_[i], z, g, d, ball_radius_, tol_);
  }

  for (auto i : utils::iota(nC)) {
    real const rho = this->rho_[i];
    auto const& g = gap_[i];
    auto const& d = dual_[i];
    m32 const rhogd = (g + d) * rho;
    ConstraintMap::ConstVisitor v = constraint_mapping_[i];
    for (idx i = 0; i < 2; ++i) {
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

void Constraint_CollidingBalls::UpdateRhoConsensus(real) {}

void Constraint_CollidingBalls::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<std::pair<idx, idx>> new_collisions;
  std::vector<idx> new_colliding_vertices;

  auto put_vert = [&](idx v) {
    if (colliding_vertices_.insert(v).second) {
      new_colliding_vertices.push_back(v);
    }
  };

  auto put_coll = [this](std::pair<idx, idx> vv) -> bool {
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
  idx const nV = g.vertices_.cols();
  for (idx i : utils::iota(nV)) {
    for (idx j: utils::iota(nV)) {
      if (i == j) continue;

      v3 const& pi = g.vertices_.col(i);
      v3 const& pj = g.vertices_.col(j);

      if (math::norm(pi - pj) < ball_radius_ + tol_ * 3) {
        put_vert(i);
        put_vert(j);
        if (put_coll({i, j})) {
          new_collisions.push_back({i, j});
        }
      }
    }
  }

  real const dt2 = g.dt_ * g.dt_;
  if (new_collisions.size() > 0) {
    for (auto v: new_colliding_vertices) {
      constrained_vertices_ids_.push_back(v);
      global_to_local_.emplace(v, constrained_vertices_ids_.size() - 1);
      constrained_vertices_position_.push_back(g.vertices_.col(v));
    }

    for (auto [i, j]: new_collisions) {
      idx const vi = global_to_local_.at(i);
      idx const vj = global_to_local_.at(j);
      constraint_mapping_.emplace_back(vi, vj);
      auto & dual = dual_.emplace_back();
      auto & gap = gap_.emplace_back();
      dual.col(0) = g.vertices_.col(i);
      dual.col(1) = g.vertices_.col(j);
      gap.setZero();
      rho_.push_back(1e2 * dt2);
      stiffness_.push_back(1e2 * dt2);

      colliding_map_.emplace(std::minmax(i, j), rho_.size() - 1);
    }
  }


  // now update the position.
  idx nCV = GetNumConstrainedVertices();
  for (idx i = 0; i < nCV; ++i) {
    idx const v = constrained_vertices_ids_[i];
    local[i] = g.vertices_.col(v);
  }
}

}