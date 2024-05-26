#include "ax/xpbd/constraints/edge_edge_collider.hpp"

#include "ax/geometry/intersection/edge_edge.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

using namespace geo;

void Constraint_EdgeEdgeCollider::BeginStep() {
  initial_rho_ = 0.1 / (tol_ * tol_);
  gap_.clear();
  dual_.clear();
  collidings_.clear();
  colliding_vertices_.clear();
  origin_.clear();
  global_to_local_.clear();
  this->constrained_vertices_ids_.clear();
  this->constrained_vertices_position_.clear();
  this->constraint_mapping_.clear();
  this->rho_.clear();
  this->rho_global_ = 1;
  iteration_ = 0;
  this->UpdatePositionConsensus();
}

using m34 = math::matr<3, 4>;
using m4 = math::mat4r;
static bool relax(m34 const& z, m34 const& u, m34 const& o, m34& x, real rho, real& k, real tol) {
  m34 const zu = z - u;
  math::vec3r normal = math::normalized(math::cross(zu.col(1) - zu.col(0), zu.col(3) - zu.col(2)));
  // TODO: If normal is zero.
  math::vec3r center = 0.25 * (zu.col(0) + zu.col(1) + zu.col(2) + zu.col(3));
  real c = math::dot(normal, center);
  real e = math::dot(normal, zu.col(3)) - c;
  if (e < 0) {
    normal = -normal;
    c = -c;
    e = -e;
  }
  auto proj = [&](auto const& p) { return p - math::dot(normal, p - center) * normal; };

  for (idx i = 0; i < 4; ++i) {
    x.col(i) = proj(zu.col(i));
  }

  if (math::dot(normal, o.col(3)) - c > 0) {
    if (e >= tol - math::epsilon<>) {
      x = zu;
      return false;
    }

    real l = (k * tol + rho * e) / (k + rho);
    x.col(0) += l * normal;
    x.col(1) += l * normal;
    x.col(2) -= l * normal;
    x.col(3) -= l * normal;
  } else {
    real l = (k * -tol + rho * e) / (k + rho);
    if (l >= -0.5 * tol) {
      k = (rho * e / tol) * 3;
      l = (k * -tol + rho * e) / (k + rho);
    }

    x.col(0) -= l * normal;
    x.col(1) -= l * normal;
    x.col(2) += l * normal;
    x.col(3) += l * normal;
  }

  return true;
}

ConstraintSolution Constraint_EdgeEdgeCollider::SolveDistributed() {
  idx const nC = GetNumConstraints();
  idx const nV = GetNumConstrainedVertices();

  ConstraintSolution sol(nV);
  for (auto i : utils::iota(nC)) {
    auto C = constraint_mapping_[i];
    m34 dual_old = dual_[i];
    auto& x = dual_[i];
    auto& u = gap_[i];
    real& k = stiffness_[i];
    real& rho = rho_[i];
    m34 z;
    for (idx i = 0; i < 4; ++i) z.col(i) = this->constrained_vertices_position_[C[i]];
    relax(z, u, origin_[i], x, rho_[i], k, tol_);
    rho *= ratio_;
    u /= ratio_;
    for (idx i = 0; i < 4; ++i) {
      sol.weighted_position_.col(C[i]) += (x.col(i) + u.col(i)) * rho;
      sol.weights_[C[i]] += rho;
    }
    sol.sqr_dual_residual_ += (x - dual_old).squaredNorm();
  }

  iteration_ += 1;
  return sol;
}

real Constraint_EdgeEdgeCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i : utils::iota(this->GetNumConstraints())) {
    auto cons = constraint_mapping_[i];
    auto const& d = dual_[i];
    math::matr<3, 4> z, du;
    for (idx i = 0; i < 4; ++i) z.col(i) = fetch_from_global[cons[i]];
    du = d - z;
    gap_[i] += du;
    sqr_prim_res += du.squaredNorm();
  }
  return sqr_prim_res;
}

void Constraint_EdgeEdgeCollider::EndStep() {}

void Constraint_EdgeEdgeCollider::UpdateRhoConsensus(real scale) {
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_EdgeEdgeCollider::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<geo::CollisionInfo> new_collisions;
  std::vector<idx> new_colliding_vertices;

  auto put_vert = [&](idx v) {
    if (colliding_vertices_.insert(v).second) {
      new_colliding_vertices.push_back(v);
    }
  };

  // Current implementation is brute force.
  for (auto [i, e1] : utils::enumerate(g.edges_)) {
    for (auto [j, e2] : utils::enumerate(g.edges_)) {
      if (i == j) continue;
      if (e1.x() == e2.x() || e1.x() == e2.y() || e1.y() == e2.x() || e1.y() == e2.y()) continue;


      auto const& e00 = g.last_vertices_.col(e1.x());
      auto const& e01 = g.last_vertices_.col(e1.y());
      auto const& e10 = g.last_vertices_.col(e2.x());
      auto const& e11 = g.last_vertices_.col(e2.y());

      auto const& e00_new = g.vertices_.col(e1.x());
      auto const& e01_new = g.vertices_.col(e1.y());
      auto const& e10_new = g.vertices_.col(e2.x());
      auto const& e11_new = g.vertices_.col(e2.y());

      auto info
          = detect_edge_edge(CollidableSegment(i, Segment3(e00, e01 - e00)),
                             CollidableSegment(i, Segment3(e00_new, e01_new - e00_new)),
                             CollidableSegment(j, Segment3(e10, e11 - e10)),
                             CollidableSegment(j, Segment3(e10_new, e11_new - e10_new)), 2 * tol_);

      if (info) {
        // has new collide.
        if (collidings_.insert(info).second) {
          new_collisions.push_back(info);
          put_vert(e1.x());
          put_vert(e1.y());
          put_vert(e2.x());
          put_vert(e2.y());
        }

        std::cout << "Collision detected: " << info.ee_a_ << " " << info.ee_b_ << " "
                  << "e0: " << e1.transpose() << " e1: " << e2.transpose() << std::endl;
        std::cout << e00.transpose() << ", " << e01.transpose() << ", " << e10.transpose() << ", "
                  << e11.transpose() << std::endl;
      }
    }
  }

  if (new_collisions.size() > 0) {
    for (auto v : new_colliding_vertices) {
      this->constrained_vertices_ids_.push_back(v);
      global_to_local_[v] = GetNumConstrainedVertices() - 1;
      this->constrained_vertices_position_.push_back(g.vertices_.col(v));
    }

    for (auto const& c : new_collisions) {
      auto e1 = g.edges_[c.ee_a_];
      auto e2 = g.edges_[c.ee_b_];
      constraint_mapping_.emplace_back(global_to_local_[e1.x()], global_to_local_[e1.y()],
                                       global_to_local_[e2.x()], global_to_local_[e2.y()]);
      auto& di = dual_.emplace_back();
      auto& actual = gap_.emplace_back();
      di.col(0) = g.last_vertices_.col(e1.x());
      actual.col(0) = g.vertices_.col(e1.x()) - g.last_vertices_.col(e1.x());
      di.col(1) = g.last_vertices_.col(e1.y());
      actual.col(1) = g.vertices_.col(e1.y()) - g.last_vertices_.col(e1.y());
      di.col(2) = g.last_vertices_.col(e2.x());
      actual.col(2) = g.vertices_.col(e2.x()) - g.last_vertices_.col(e2.x());
      di.col(3) = g.last_vertices_.col(e2.y());
      actual.col(3) = g.vertices_.col(e2.y()) - g.last_vertices_.col(e2.y());

      origin_.emplace_back(di);
      stiffness_.push_back(initial_rho_ * g.dt_ * g.dt_);
      rho_.push_back(initial_rho_ * g.dt_ * g.dt_);
    }
  }

  idx n_v = this->GetNumConstrainedVertices();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd
