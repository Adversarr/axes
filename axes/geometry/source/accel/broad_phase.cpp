#include "ax/geometry/accel/broad_phase.hpp"

namespace ax::geo {

void BroadPhaseBase::ExpandAABB(Real epsilon) {
  for (auto& c : colliders_) {
    c.aabb_.min().array() -= epsilon;
    c.aabb_.max().array() += epsilon;
  }
}

BroadPhaseResult const& BroadPhaseBase::GetCollidingPairs() const { return collidings_; }

void BroadPhaseBase::Reserve(size_t n) { colliders_.reserve(n); }

void BroadPhaseBase::AddCollidingPair(Index a, Index b) {
  auto const& ca = GetCollider(a);
  auto const& cb = GetCollider(b);
  auto kind = get_collision_kind(ca.external_kind_, cb.external_kind_);
  BroadPhaseCollisionInfo ci;
  ci.a_ = a;
  ci.b_ = b;
  ci.info_.valid_ = true;
  ci.info_.type_ = kind;
  collidings_[kind].push_back(ci);
}
}  // namespace ax::geo
