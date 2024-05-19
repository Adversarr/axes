#pragma once
#include <boost/describe/enum.hpp>

#include "ax/math/common.hpp"
#include "ax/utils/common.hpp"

namespace ax::geo {

BOOST_DEFINE_ENUM_CLASS(CollisionKind, kVertexFace, kVertexEdge, kVertexVertex, kEdgeEdge);

struct CollisionInfo {
  bool valid_;
  CollisionKind type_;
  real rel_t_;
  union {
    idx vf_vertex_;
    idx ve_vertex_;
    idx vv_a_;
    idx ee_a_;
  };

  union {
    idx vf_face_;
    idx ve_edge_;
    idx vv_b_;
    idx ee_b_;
  };

  inline CollisionInfo()
      : valid_(false), type_(CollisionKind::kVertexFace), rel_t_(0), vf_vertex_(-1), vf_face_(-1) {}

  operator std::pair<idx, idx>() const { return {vf_vertex_, vf_face_}; }

  CollisionKind GetKind() const { return type_; }

  operator bool() const { return valid_; }

  inline static CollisionInfo VertexFace(idx vertex, idx face) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kVertexFace;
    info.vf_vertex_ = vertex;
    info.vf_face_ = face;
    return info;
  }

  inline static CollisionInfo VertexEdge(idx vertex, idx edge) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kVertexEdge;
    info.ve_vertex_ = vertex;
    info.ve_edge_ = edge;
    return info;
  }

  inline static CollisionInfo VertexVertex(idx a, idx b) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kVertexVertex;
    info.vv_a_ = a;
    info.vv_b_ = b;
    return info;
  }

  inline static CollisionInfo EdgeEdge(idx a, idx b) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kEdgeEdge;
    info.ee_a_ = a;
    info.ee_b_ = b;
    return info;
  }
};

template <idx dim>
struct Segment {
  idx id_;
  math::vecr<dim> start_, end_;
  AX_HOST_DEVICE Segment(idx id, math::vecr<dim> const& start, math::vecr<dim> const& end)
      : id_(id), start_(start), end_(end) {}
};

template <idx dim>
struct Vertex {
  idx id_;
  math::vecr<dim> position_;
  AX_HOST_DEVICE Vertex(idx id, math::vecr<dim> const& position) : id_(id), position_(position) {}
};

template <idx dim>
struct Face {
  idx id_;
  std::array<math::vecr<dim>, dim> vertices_;
  template <typename ... Args>
  AX_HOST_DEVICE Face(idx id, Args&& ... args) : id_(id), vertices_{std::forward<Args>(args)...} {}
};

}  // namespace ax::geo