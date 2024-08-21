#pragma once
#include <Eigen/Geometry>
#include <map>

#include "ax/core/logging.hpp"
#include "ax/utils/enum_refl.hpp"
#include "ax/geometry/common.hpp"  // IWYU pragma: export
namespace ax::geo {

/****************************** AABB ******************************/

template <int dim> using AlignedBoxN = Eigen::AlignedBox<Real, dim>;

using AlignedBox2 = AlignedBoxN<2>;
using AlignedBox3 = AlignedBoxN<3>;

AX_HOST_DEVICE AX_FORCE_INLINE AlignedBox2 to_aabb(Vertex2 const& center) {
  return AlignedBox2(center.Position(), center.Position());
}

AX_HOST_DEVICE AX_FORCE_INLINE AlignedBox3 to_aabb(Vertex3 const& center) {
  return AlignedBox3(center.Position(), center.Position());
}

AX_HOST_DEVICE AX_FORCE_INLINE AlignedBox2 to_aabb(Segment2 const& line) {
  return AlignedBox2(line.Origin(), line.End());
}

AX_HOST_DEVICE AX_FORCE_INLINE AlignedBox3 to_aabb(Segment3 const& line) {
  return AlignedBox3(line.Origin(), line.End());
}

AX_HOST_DEVICE AX_FORCE_INLINE AlignedBox2 to_aabb(Triangle2 const& triangle) {
  math::RealVector2 min = triangle.A().cwiseMin(triangle.B()).cwiseMin(triangle.C());
  math::RealVector2 max = triangle.A().cwiseMax(triangle.B()).cwiseMax(triangle.C());
  return AlignedBox2(min, max);
}

AX_HOST_DEVICE AX_FORCE_INLINE AlignedBox3 to_aabb(Triangle3 const& triangle) {
  math::RealVector3 min = triangle.A().cwiseMin(triangle.B()).cwiseMin(triangle.C());
  math::RealVector3 max = triangle.A().cwiseMax(triangle.B()).cwiseMax(triangle.C());
  return AlignedBox3(min, max);
}

template <int dim> AX_HOST_DEVICE AX_FORCE_INLINE bool has_collide(AlignedBoxN<dim> const& a,
                                                                   AlignedBoxN<dim> const& b) {
  return a.intersects(b);
}

AX_DEFINE_ENUM_CLASS(CollisionKind, kVertexFace, kVertexEdge, kVertexVertex, kEdgeEdge, kNone);

struct CollisionInfo {
  bool valid_;
  CollisionKind type_;
  Real rel_t_;

  inline CollisionInfo() : valid_(false), type_(CollisionKind::kVertexFace), rel_t_(0) {}

  AX_DECLARE_CONSTRUCTOR(CollisionInfo, default, default);

  CollisionKind GetKind() const { return type_; }

  operator bool() const { return valid_; }

  inline static CollisionInfo VertexFace(Real rel_t) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kVertexFace;
    info.rel_t_ = rel_t;
    return info;
  }

  inline static CollisionInfo VertexEdge(Real rel_t) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kVertexEdge;
    info.rel_t_ = rel_t;
    return info;
  }

  inline static CollisionInfo VertexVertex(Real rel_t) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kVertexVertex;
    info.rel_t_ = rel_t;
    return info;
  }

  inline static CollisionInfo EdgeEdge(Real rel_t) {
    CollisionInfo info;
    info.valid_ = true;
    info.type_ = CollisionKind::kEdgeEdge;
    info.rel_t_ = rel_t;
    return info;
  }
};

struct BroadPhaseCollisionInfo {
  Index a_;
  Index b_;
  CollisionInfo info_;
};

/************************* SECT: Broad Phase *************************/
AX_DEFINE_ENUM_CLASS(PrimitiveKind, kVertex, kSegment, kTriangle);

namespace details {

template <typename T> struct primitive_kind_refl;

template <> struct primitive_kind_refl<Vertex3> {
  static constexpr PrimitiveKind value = PrimitiveKind::kVertex;
};
template <> struct primitive_kind_refl<Segment3> {
  static constexpr PrimitiveKind value = PrimitiveKind::kSegment;
};
template <> struct primitive_kind_refl<Triangle3> {
  static constexpr PrimitiveKind value = PrimitiveKind::kTriangle;
};
}  // namespace details

template <typename T> constexpr PrimitiveKind primitive_kind_refl_v
    = details::primitive_kind_refl<std::decay_t<T>>::value;

struct ColliderInfo {
  AlignedBox3 aabb_;
  Index external_id_;
  Index parent_id_;
  PrimitiveKind external_kind_;
};

using CollidingPair = std::pair<Index, Index>;

using BroadPhaseResult = std::map<CollisionKind, std::vector<BroadPhaseCollisionInfo>>;

inline CollisionKind get_collision_kind(PrimitiveKind a, PrimitiveKind b) {
  if (static_cast<int>(b) < static_cast<int>(a)) return get_collision_kind(b, a);
  if (a == PrimitiveKind::kVertex && b == PrimitiveKind::kTriangle) {
    return CollisionKind::kVertexFace;
  } else if (a == PrimitiveKind::kVertex && b == PrimitiveKind::kSegment) {
    return CollisionKind::kVertexEdge;
  } else if (a == PrimitiveKind::kVertex && b == PrimitiveKind::kVertex) {
    return CollisionKind::kVertexVertex;
  } else if (a == PrimitiveKind::kSegment && b == PrimitiveKind::kSegment) {
    return CollisionKind::kEdgeEdge;
  }
  AX_UNREACHABLE();
}

/************************* SECT: narrow phase *************************/

}  // namespace ax::geo
