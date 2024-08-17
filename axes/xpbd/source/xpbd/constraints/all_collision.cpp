#include "ax/xpbd/constraints/all_collision.hpp"

#include <cassert>

#include "ax/geometry/intersection/edge_edge.hpp"
#include "ax/geometry/intersection/vertex_face.hpp"
#include "ax/utils/iota.hpp"

namespace ax::xpbd {

using namespace geo;
// two main collision:
// 1. vertex-to-triangle
// 2. edge-to-edge
// for each collision, we need to determine the real collision type:
// 1. v-t
// 2. v-e
// 3. v-v
// 3. e-e
// for each type of collision, we need to define the `relax` method.

using I4 = math::IndexVec4;
using v3 = math::RealVector3;

std::pair<CollisionKind, I4> determine_collision(v3 const& v_start, v3 const& v_end,
                                                 v3 const& f0_start, v3 const& f1_start,
                                                 v3 const& f2_start, v3 const& f0_end,
                                                 v3 const& f1_end, v3 const& f2_end, real tol) {
  // Need to determine: v-t? v-v? v-e?
  // question is, should this be a CCD?
}

ConstraintSolution Constraint_AllCollision::SolveDistributed() {
  Index const nV = GetNumConstrainedVertices();
  ConstraintSolution solution(nV);
  std::map<CollisionKind, std::vector<I4>> collisions_to_resolve;

  return solution;
}

void Constraint_AllCollision::BeginStep() {}

void Constraint_AllCollision::EndStep() {}

void Constraint_AllCollision::UpdatePositionConsensus() {
  auto& g = ensure_server();

  std::vector<I2> new_vt, new_ee;

  // detect vt ee
  Index const nV = g.vertices_.cols(), nT = static_cast<Index>(g.faces_.size()),
            nE = static_cast<Index>(g.edges_.size());

  size_t const num_last_vt = vt_.size(), num_last_ee = ee_.size();

  auto put_vt_collision = [&](Index v, Index f) {
    if (auto it = vt_.find({v, f}); it == vt_.end()) {
      vt_.insert({v, f});
      new_vt.push_back({v, f});
    }
  };

  auto put_ee_collision = [&](Index e1, Index e2) {
    if (auto it = ee_.find({e1, e2}); it == ee_.end()) {
      ee_.insert({e1, e2});
      new_ee.push_back({e1, e2});
    }
  };
  auto put_vertex = [&](Index v) -> bool {
    if (auto it = colliding_vertices_.find(v); it == colliding_vertices_.end()) {
      colliding_vertices_.insert(v);
      global_to_local_[v] = GetNumConstrainedVertices();
      constrained_vertices_ids_.push_back(v);
      constrained_vertices_position_.emplace_back();
      return true;
    } else {
      return false;
    }
  };

  for (Index i = 0; i < nV; ++i) {
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

      auto info = detect_vertex_face(Vertex3{x0}, Vertex3{x1}, Triangle3{fx0, fy0, fz0},
                                     Triangle3{fx1, fy1, fz1}, 2 * tol_);

      if (info) {
        put_vt_collision(i, j);
        put_vertex(i);
        for (Index v : f) put_vertex(v);
      }
    }
  }

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

      auto info = detect_edge_edge(Segment3(e00, e01 - e00), Segment3(e00_new, e01_new - e00_new),
                                   Segment3(e10, e11 - e10), Segment3(e10_new, e11_new - e10_new),
                                   2 * tol_);

      if (info) {
        // has new collide.
        put_ee_collision(i, j);
        put_vertex(e1.x());
        put_vertex(e1.y());
        put_vertex(e2.x());
        put_vertex(e2.y());
      }
    }
  }

  // Fetch all the vertex position normally.
  ConstraintBase::UpdatePositionConsensus();

  auto query = [this](Index global) { return global_to_local_.at(global); };

  for (auto [v, f] : new_vt) {
    auto const& face = g.faces_[f];
    this->constraint_mapping_.emplace_back(query(v), query(face.x()), query(face.y()),
                                           query(face.z()));
    kind_.push_back(CollisionKind::kVertexFace);
    auto& di = dual_.emplace_back();
    auto& actual = gap_.emplace_back();

    di.col(0) = g.last_vertices_.col(v);
    actual.col(0) = g.vertices_.col(v) - g.last_vertices_.col(v);
    for (auto [i, v] : utils::enumerate(face)) {
      di.col(i + 1) = g.last_vertices_.col(v);
      actual.col(i + 1) = g.vertices_.col(v) - g.last_vertices_.col(v);
    }
    real const mass = g.mass_[v];
    origin_.emplace_back(di);
    stiffness_.push_back(mass * 10);
    rho_.push_back(mass * 10);
  }

  for (auto [e1, e2] : new_ee) {
    auto const& edge1 = g.edges_[e1];
    auto const& edge2 = g.edges_[e2];
    constraint_mapping_.emplace_back(query(edge1.x()), query(edge1.y()), query(edge2.x()),
                                     query(edge2.y()));
    kind_.push_back(CollisionKind::kEdgeEdge);
    auto& di = dual_.emplace_back();
    auto& actual = gap_.emplace_back();
    di.col(0) = g.last_vertices_.col(edge1.x());
    actual.col(0) = g.vertices_.col(edge1.x()) - g.last_vertices_.col(edge1.x());
    di.col(1) = g.last_vertices_.col(edge1.y());
    actual.col(1) = g.vertices_.col(edge1.y()) - g.last_vertices_.col(edge1.y());
    di.col(2) = g.last_vertices_.col(edge2.x());
    actual.col(2) = g.vertices_.col(edge2.x()) - g.last_vertices_.col(edge2.x());
    di.col(3) = g.last_vertices_.col(edge2.y());
    actual.col(3) = g.vertices_.col(edge2.y()) - g.last_vertices_.col(edge2.y());

    origin_.emplace_back(di);
    stiffness_.push_back(initial_rho_ * g.dt_ * g.dt_);
    rho_.push_back(initial_rho_ * g.dt_ * g.dt_);
  }
}

}  // namespace ax::xpbd
