#include "ax/xpbd/constraints/vertex_face_collider.hpp"

#include "ax/geometry/intersection/vertex_edge.hpp"
#include "ax/geometry/intersection/vertex_face.hpp"
#include "ax/geometry/intersection/vertex_vertex.hpp"
#include "ax/utils/ndrange.hpp"
#include "ax/xpbd/details/relaxations.hpp"
#include <range/v3/view/enumerate.hpp>

namespace ax::xpbd {
using namespace geo;

void Constraint_VertexFaceCollider::BeginStep() {
  initial_rho_ = 1 / (tol_ * tol_);
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

std::pair<CollisionKind, Index> determine_real_collision_kind(m34 const& o, Real tol) {
  Vertex3 v{o.col(0)}, f1{o.col(1)}, f2{o.col(2)}, f3{o.col(3)};
  Segment3 e12{o.col(1), o.col(2) - o.col(1)}, e23{o.col(2), o.col(3) - o.col(2)},
      e31{o.col(3), o.col(1) - o.col(3)};
  Triangle3 t{o.col(1), o.col(2), o.col(3)};
  CollisionInfo vf
      = detect_vertex_face(Vertex3{o.col(0)}, Triangle3(o.col(1), o.col(2), o.col(3)), tol);
  CollisionInfo ve12 = detect_vertex_edge(v, e12, tol);
  CollisionInfo ve23 = detect_vertex_edge(v, e23, tol);
  CollisionInfo ve31 = detect_vertex_edge(v, e31, tol);
  CollisionInfo vv1 = detect_vertex_vertex(v, f1, tol);
  CollisionInfo vv2 = detect_vertex_vertex(v, f2, tol);
  CollisionInfo vv3 = detect_vertex_vertex(v, f3, tol);

  // std::cout << std::boolalpha << "vf: " <<  static_cast<bool>(vf) << std::endl;
  // std::cout << std::boolalpha << "vv1: " <<  static_cast<bool>(vv1) << std::endl;
  // std::cout << std::boolalpha << "vv2: " <<  static_cast<bool>(vv2) << std::endl;
  // std::cout << std::boolalpha << "vv3: " <<  static_cast<bool>(vv3) << std::endl;
  // std::cout << std::boolalpha << "ve12: " <<  static_cast<bool>(ve12) << std::endl;
  // std::cout << std::boolalpha << "ve23: " <<  static_cast<bool>(ve23) << std::endl;
  // std::cout << std::boolalpha << "ve31: " <<  static_cast<bool>(ve31) << std::endl;
  //
  // NOTE: Should be changed to continuous version?
  bool any_vv = vv1 || vv2 || vv3;
  bool any_ve = ve12 || ve23 || ve31;
  if (vf) {
    return {CollisionKind::VertexFace, 0};
  } else if (any_vv) {
    if (vv1) return {CollisionKind::VertexVertex, 1};
    if (vv2) return {CollisionKind::VertexVertex, 2};
    if (vv3) return {CollisionKind::VertexVertex, 3};
  } else if (any_ve) {
    if (ve12) return {CollisionKind::VertexEdge, 12};
    if (ve23) return {CollisionKind::VertexEdge, 23};
    if (ve31) return {CollisionKind::VertexEdge, 31};
  } else {
    // actually, cannot determine?
    return {CollisionKind::None, 0};
  }
  AX_UNREACHABLE();
}

ConstraintSolution Constraint_VertexFaceCollider::SolveDistributed() {
  Index const nC = GetNumConstraints();
  Index const nV = GetNumConstrainedVertices();

  // AX_LOG(ERROR) << "Number of constraints: " << nC;
  ConstraintSolution sol(nV);
  for (auto i : utils::range(nC)) {
    auto C = constraint_mapping_[i];
    m34 dual_old = dual_[i];
    auto& x = dual_[i];
    auto& u = gap_[i];  // u.setZero();
    Real& k = stiffness_[i];
    Real& rho = rho_[i];
    m34 z;
    for (Index i = 0; i < 4; ++i) z.col(i) = this->constrained_vertices_position_[C[i]];
    auto [kind, id] = determine_real_collision_kind(origin_[i], tol_);
    if (kind == geo::CollisionKind::VertexVertex) {
      AX_LOG(ERROR) << utils::reflect_name(kind).value_or("?") << " " << id;
      m32 z_vv, u_vv, o_vv;
      z_vv.col(0) = z.col(0);
      z_vv.col(1) = z.col(id);
      u_vv.col(0) = u.col(0);
      u_vv.col(1) = u.col(id);
      o_vv.col(0) = origin_[i].col(0);
      o_vv.col(1) = origin_[i].col(id);
      m32 x_vv;
      relax_vertex_vertex_impl(z_vv, u_vv, o_vv, x_vv, k, rho, 0.2 * tol_, 0.05 * tol_);
      x.col(0) = x_vv.col(0);
      x.col(id) = x_vv.col(1);
      for (Index i = 1; i < 4; ++i) {
        if (i != id) x.col(i) = (z - u).col(i);
      }
    } else if (kind == geo::CollisionKind::VertexEdge) {
      AX_LOG(ERROR) << utils::reflect_name(kind).value_or("?") << " " << id;
      m3 z_ve, u_ve;
      Index unused = 6 - (id / 10) % 10 - id % 10;
      z_ve.col(0) = z.col(0);
      z_ve.col(1) = z.col((id / 10) % 10);
      z_ve.col(2) = z.col(id % 10);
      u_ve.col(0) = u.col(0);
      u_ve.col(1) = u.col((id / 10) % 10);
      u_ve.col(2) = u.col(id % 10);
      m3 x_ve, o_ve;
      o_ve.col(0) = origin_[i].col(0);
      o_ve.col(1) = origin_[i].col((id / 10) % 10);
      o_ve.col(2) = origin_[i].col(id % 10);
      std::cout << o_ve << std::endl;
      relax_vertex_edge_impl(z_ve, u_ve, o_ve, x_ve, rho, k, tol_ * 0.2);
      x.col(0) = x_ve.col(0);
      x.col((id / 10) % 10) = x_ve.col(1);
      x.col(id % 10) = x_ve.col(2);
      x.col(unused) = (z - u).col(unused);
      for (Index i = 0; i < 4; ++i) {
        if (i == unused) continue;
        sol.weighted_position_.col(C[i]) += (x.col(i) + u.col(i)) * rho;
        sol.weights_[C[i]] += rho;
      }
    } else if (kind == geo::CollisionKind::VertexFace) {
      relax_vertex_triangle_impl(z, u, origin_[i], x, rho_[i], k, tol_);
      for (Index i = 0; i < 4; ++i) {
        sol.weighted_position_.col(C[i]) += (x.col(i) + u.col(i)) * rho;
        sol.weights_[C[i]] += rho;
      }
    } else {
      x = z - u;
      // TODO: ???????
    }
    sol.sqr_dual_residual_ += (x - dual_old).squaredNorm();
    rho *= ratio_;
    u /= ratio_;
  }

  iteration_ += 1;
  return sol;
}

Real Constraint_VertexFaceCollider::UpdateDuality() {
  auto const& fetch_from_global = this->constrained_vertices_position_;
  Real sqr_prim_res = 0;
  for (auto i : utils::range(this->GetNumConstraints())) {
    auto cons = constraint_mapping_[i];
    auto const& d = dual_[i];
    math::RealMatrix<3, 4> z, du;
    for (Index i = 0; i < 4; ++i) z.col(i) = fetch_from_global[cons[i]];
    du = d - z;
    gap_[i] += du;
    sqr_prim_res += du.squaredNorm();
  }
  return sqr_prim_res;
}

void Constraint_VertexFaceCollider::EndStep() {
  for (auto [vf, n] : collidings_) {
    std::cout << "Collision: " << vf.first << " " << vf.second << " " << n << std::endl;
  }
}

void Constraint_VertexFaceCollider::UpdateRhoConsensus(Real scale) {
  for (auto& r : this->rho_) r *= scale;
  this->rho_global_ *= scale;
  for (auto& g : gap_) g /= scale;
}

void Constraint_VertexFaceCollider::UpdatePositionConsensus() {
  auto const& cmap = this->constrained_vertices_ids_;
  auto& local = this->constrained_vertices_position_;
  auto const& g = ensure_server();
  std::vector<std::pair<Index, Index>> new_collisions;
  std::vector<Index> new_colliding_vertices;

  auto put_vert = [&](Index v) {
    if (colliding_vertices_.insert(v).second) {
      new_colliding_vertices.push_back(v);
    }
  };

  auto put_coll = [this](std::pair<Index, Index> vf) -> bool {
    auto it = collidings_.find(vf);
    if (it == collidings_.end()) {
      collidings_.emplace(vf, 1);
      return true;
    } else {
      // auto i = colliding_map_.at(vf);
      // rho_[i] *= ratio_;
      // std::cout << "Update " << i << " " << rho_[i] << std::endl;
      it->second += 1;
      return false;
    }
  };

  // Current implementation is brute force.
  for (Index i : utils::range(g.vertices_.cols())) {
    for (auto [j, f] : utils::views::enumerate(g.faces_)) {
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
        // has new collide.
        if (put_coll({i, j})) {
          new_collisions.push_back({i, j});
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

    for (auto const& [vid, fid] : new_collisions) {
      auto f = g.faces_[fid];
      this->constraint_mapping_.emplace_back(global_to_local_[vid], global_to_local_[f.x()],
                                             global_to_local_[f.y()], global_to_local_[f.z()]);
      auto& di = dual_.emplace_back();
      gap_.emplace_back().setZero();
      di.col(0) = g.last_vertices_.col(vid);
      for (auto [i, v] : utils::views::enumerate(f)) {
        di.col(i + 1) = g.last_vertices_.col(v);
      }
      Real const mass = 0.25 * g.mass_[vid] + 0.25 * g.mass_[f.x()] + 0.25 * g.mass_[f.y()]
                        + 0.25 * g.mass_[f.z()];

      origin_.emplace_back(di);
      stiffness_.push_back(mass * 10);
      rho_.push_back(mass * 10);
    }
  }

  Index n_v = this->GetNumConstrainedVertices();
  for (Index i : utils::range(n_v)) {
    Index iV = cmap[i];
    local[i] = g.vertices_.col(iV);
  }
}

}  // namespace ax::xpbd
