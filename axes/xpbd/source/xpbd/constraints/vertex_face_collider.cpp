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
  m4 expect;
  m34 const zu = z - u;
  expect.topRows<3>() = z - u;
  expect.bottomRows<1>().setOnes();

  m4 origin;
  origin.topRows<3>() = o;
  origin.bottomRows<1>().setOnes();
  real det_expect = expect.determinant(), det_origin = origin.determinant();
  std::cout << "det_expect: " << det_expect << " det_origin: " << det_origin << std::endl;
  if (det_expect * det_origin > 0) {
    x = zu;
    return false;
  }
  // there exist the collision.
  // step 1: find the seperating plane.
  math::vec3r normal = math::normalized(math::cross(zu.col(2) - zu.col(1), zu.col(3) - zu.col(1)));
  math::vec3r center = 0.25 * (zu.col(0) + zu.col(1) + zu.col(2) + zu.col(3));
  real c = math::dot(normal, center);
  real e = math::dot(normal, zu.col(0)) - c;
  if (e > 0) {
    normal = -normal;
    c = -c;
    e = -e;
  }

  // project to the plane.
  auto proj = [&](math::vec3r const& p) -> math::vec3r {
    return p - math::dot(normal, p - center) * normal;
  };
  for (idx i = 0; i < 4; ++i) {
    x.col(i) = proj(zu.col(i));
  }

  // step 2: solve vertex.
  real l = (k * tol + rho * e) / (k + rho);
  if (l < 0) {
    // We need to enforce the constraint strictly, l >= 0.
    // (k t + r e) > 0 => k >= -r e / t
    k = -4 * (rho * e / tol);
    l = (k * tol + rho * e) / (k + rho);
  }

  x.col(0) += l * normal;
  x.col(1) -= l * normal / 3;
  x.col(2) -= l * normal / 3;
  x.col(3) -= l * normal / 3;
  return true;
}

ConstraintSolution Constraint_VertexFaceCollider::SolveDistributed() {
  idx const nC = GetNumConstraints();
  idx const nV = GetNumConstrainedVertices();

  ConstraintSolution sol(nV);
  for (auto i : utils::iota(nC)) {
    auto C = constraint_mapping_[i];
    auto& z = dual_[i];
    auto const& u = gap_[i];
    real& k = stiffness_[i];
    real& rho = rho_[i];
    math::matr<3, 4> x;
    for (idx i = 0; i < 4; ++i) x.col(i) = this->constrained_vertices_position_[C[i]];
    bool has_collide = relax(x, u, origin_[i], z, rho_[i], k, tol_);
    rho = k;
    if (has_collide) {
      std::cout << "Still collide, " << iteration_ << " " << i << std::endl;
    }

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
      auto const& x = g.vertices_.col(i);
      auto const& fx = g.vertices_.col(f.x());
      auto const& fy = g.vertices_.col(f.y());
      auto const& fz = g.vertices_.col(f.z());

      if (i == f.x() || i == f.y() || i == f.z()) continue;

      auto info = detect_vertex_face(CollidableVertex(i, Vertex3{x}),
                                     CollidableTriangle(j, Triangle3{fx, fy, fz}), 0.01);

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
      this->constrained_vertices_position_.push_back(g.vertices_.col(v));
    }

    for (auto const& c : new_collisions) {
      auto f = g.faces_[c.vf_face_];
      this->constraint_mapping_.emplace_back(c.vf_vertex_, f.x(), f.y(), f.z());
      auto& di = dual_.emplace_back();
      di.col(0) = g.last_vertices_.col(c.vf_vertex_);
      for (auto [i, v] : utils::enumerate(f)) {
        di.col(i + 1) = g.last_vertices_.col(v);
      }

      origin_.emplace_back(di);
      gap_.emplace_back().setZero();
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
