#include "ax/xpbd/constraints/vertex_face_collider.hpp"

#include "ax/geometry/intersection/vertex_face.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {
using namespace geo;

void Constraint_VertexFaceCollider::BeginStep() {
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
using m4 = math::matr<4, 4>;

bool relax(m34 const& z, m34 const& u, m34 const& o, m34& x, real rho, real& k, real tol) {
  // k/2(|| Dx || - d)^2 + rho/2 ||x-z+u||^2
  // first, test if there is a collision currently.
  // if there is, then we need to solve the problem.
  m34 const zu = z - u;
  // there exist the collision.
  // step 1: find the seperating plane.
  math::vec3r normal = math::normalized(math::cross(zu.col(2) - zu.col(1), zu.col(3) - zu.col(1)));
  math::vec3r center = 0.25 * (zu.col(0) + zu.col(1) + zu.col(2) + zu.col(3));
  real c = math::dot(normal, center);
  real e = math::dot(normal, zu.col(0)) - c;
  if (e < 0) {
    normal = -normal;
    c = -c;
    e = -e;
  }

  real x0c = math::dot(normal, o.col(0)) - c;
  // project to the plane.
  auto proj = [&](math::vec3r const& p) -> math::vec3r {
    return p - math::dot(normal, p - center) * normal;
  };
  for (idx i = 0; i < 4; ++i) {
    x.col(i) = proj(zu.col(i));
  }

  // std::cout << "x0c: " << x0c << std::endl;
  // std::cout << "e: " << e << std::endl;
  if (x0c > 0) {
    // step 2: solve edge.
    if (e >= tol - math::epsilon<>) {
      x = zu;
      return false;
    }
    real l = (k * tol + rho * e) / (k + rho);
    x.col(0) += l * normal;
    x.col(1) -= l * normal / 3;
    x.col(2) -= l * normal / 3;
    x.col(3) -= l * normal / 3;
  } else {
    // step 2: solve vertex.
    real l = (k * -tol + rho * e) / (k + rho);
    if (l >= -0.5 * tol) {
      // We need to enforce the constraint strictly, l <= 0
      // (k * (-t) + r e) < 0 => k > r e / t
      k = (rho * e / tol) * 3;
      l = (k * -tol + rho * e) / (k + rho);
    }
  
    x.col(0) += l * normal;
    x.col(1) -= l * normal / 3;
    x.col(2) -= l * normal / 3;
    x.col(3) -= l * normal / 3;
  }
  return true;
}

ConstraintSolution Constraint_VertexFaceCollider::SolveDistributed() {
  idx const nC = GetNumConstraints();
  idx const nV = GetNumConstrainedVertices();

  ConstraintSolution sol(nV);
  for (auto i : utils::iota(nC)) {
    auto C = constraint_mapping_[i];
    auto& z = dual_[i];
    auto & u = gap_[i];
    real& k = stiffness_[i];
    real const k_old = k;
    real& rho = rho_[i];
    math::matr<3, 4> x;
    for (idx i = 0; i < 4; ++i) x.col(i) = this->constrained_vertices_position_[C[i]];
    bool has_collide = relax(x, u, origin_[i], z, rho_[i], k, tol_);
    if (has_collide) {
      rho *= ratio_ * k / k_old;
      u /= ratio_ * k / k_old;
    }
    std::cout << "iter: " << iteration_ << "k: " << k << "rho: " << rho << std::endl;
    for (idx i = 0; i < 4; ++i) {
      sol.weighted_position_.col(C[i]) += (z.col(i) + u.col(i)) * rho;
      sol.weights_[C[i]] += rho;
    }
  }

  iteration_ += 1;
  return sol;
}

real Constraint_VertexFaceCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i : utils::iota(this->GetNumConstraints())) {
    auto cons = constraint_mapping_[i];
    auto const& d = dual_[i];
    math::matr<3, 4> z, du;
    for (idx i = 0; i < 4; ++i) z.col(i) = fetch_from_global[cons[i]];
    du = d - z;
    gap_[i] += du;
  }
  return sqr_prim_res;
}

void Constraint_VertexFaceCollider::EndStep() {}

void Constraint_VertexFaceCollider::UpdateRhoConsensus(real scale) {
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_VertexFaceCollider::UpdatePositionConsensus() {
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
  for (idx i : utils::iota(g.vertices_.cols())) {
    for (auto [j, f] : utils::enumerate(g.faces_)) {
      auto const& x0 = g.last_vertices_.col(i);
      auto const& fx0 = g.last_vertices_.col(f.x());
      auto const& fy0 = g.last_vertices_.col(f.y());
      auto const& fz0 = g.last_vertices_.col(f.z());
      auto const& x1 = g.vertices_.col(i);
      auto const& fx1 = g.vertices_.col(f.x());
      auto const& fy1 = g.vertices_.col(f.y());
      auto const& fz1 = g.vertices_.col(f.z());

      if (i == f.x() || i == f.y() || i == f.z()) continue;

      auto info
          = detect_vertex_face(CollidableVertex(i, Vertex3{x0}), CollidableVertex(i, Vertex3{x1}),
                               CollidableTriangle(j, Triangle3{fx0, fy0, fz0}),
                               CollidableTriangle(j, Triangle3{fx1, fy1, fz1}), 2 * tol_);

      if (info) {
        // has new collide.
        if (collidings_.insert(info).second) {
          new_collisions.push_back(info);
          put_vert(i);
          for (auto v : f) put_vert(v);
        }
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
      auto f = g.faces_[c.vf_face_];
      this->constraint_mapping_.emplace_back(global_to_local_[c.vf_vertex_],
                                             global_to_local_[f.x()],
                                             global_to_local_[f.y()],
                                             global_to_local_[f.z()]);
      auto& di = dual_.emplace_back();
      auto& actual = gap_.emplace_back();
      di.col(0) = g.last_vertices_.col(c.vf_vertex_);
      actual.col(0) = g.vertices_.col(c.vf_vertex_) - g.last_vertices_.col(c.vf_vertex_);
      for (auto [i, v] : utils::enumerate(f)) {
        di.col(i + 1) = g.last_vertices_.col(v);
        actual.col(i + 1) = g.vertices_.col(v) - g.last_vertices_.col(v);
      }

      origin_.emplace_back(di);

      stiffness_.push_back(initial_rho_);
      rho_.push_back(initial_rho_);
    }
  }

  idx n_v = this->GetNumConstrainedVertices();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd
