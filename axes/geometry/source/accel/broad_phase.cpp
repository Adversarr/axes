#include "ax/geometry/accel/broad_phase.hpp"

namespace ax::geo {

void BroadPhaseBase::ExpandAABB(real epsilon) {
  for (auto& c : colliders_) {
    c.aabb_.min().array() -= epsilon;
    c.aabb_.max().array() += epsilon;
  }
}

BroadPhaseResult const& BroadPhaseBase::GetCollidingPairs() const { return collidings_; }

void BroadPhaseBase::Reserve(size_t n) { colliders_.reserve(n); }

void BroadPhaseBase::AddCollidingPair(idx a, idx b) {
  auto ci = make_collision(GetCollider(a), GetCollider(b));
  collidings_[ci.GetKind()].push_back(ci);
}
}  // namespace ax::geo
