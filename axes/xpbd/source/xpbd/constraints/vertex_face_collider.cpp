#include "ax/xpbd/constraints/vertex_face_collider.hpp"

#include "ax/geometry/intersection/vertex_face.hpp"
#include "ax/math/linalg.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

void Constraint_VertexFaceCollider::BeginStep() {
  gap_.clear();
  dual_.clear();
  collidings_.clear();
  this->constrained_vertices_ids_.clear();
  this->constraint_mapping_.clear();
  this->constrained_vertices_position_.clear();
  this->rho_.clear();
  this->rho_global_ = 1;
  iteration_ = 0;

  this->UpdatePositionConsensus();
}

real Constraint_VertexFaceCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  real sqr_prim_res = 0;
  for (auto i : utils::iota(this->GetNumConstraints())) {
    math::vec3r du = dual_[i] - fetch_from_global[i];
    gap_[i] += du;
    sqr_prim_res += math::norm2(du);
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
      math::vec3r const x = g.vertices_.col(i);
      auto info = geo::detect_vertex_face(
          geo::Vertex(i, x),
          geo::Face(j, g.vertices_.col(f.x()), g.vertices_.col(f.y()), g.vertices_.col(f.z())),
          tol_);

      if (info) {
        // has collide.
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

    for (auto const& c: new_collisions) {
      auto f = g.faces_[c.vf_face_];
      this->constraint_mapping_.emplace_back(c.vf_vertex_, f.x(), f.y(), f.z());
      auto& di = dual_.emplace_back();
      di.col(0) = g.vertices_.col(c.vf_vertex_);
      for (auto [i, v] : utils::enumerate(f)) {
        di.col(i + 1) = g.vertices_.col(v);
      }

      gap_.push_back(math::vec3r::Zero());
      stiffness_.push_back(initial_rho_);
      this->rho_.push_back(initial_rho_);
      auto& plane = seperating_plane_.emplace_back();
      plane.head<3>() = math::normalized(math::cross(di.col(2) - di.col(1), di.col(3) - di.col(1)));
      plane(3) = math::dot(plane.head<3>(), di.col(1));
      if (math::dot(plane.head<3>(), di.col(0)) < plane(3)) {
        plane(3) -= tol_;
      } else {
        plane(3) += tol_;
      }
    }
  }

  idx n_v = this->GetNumConstrainedVertices();
  for (idx i : utils::iota(n_v)) {
    idx iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd