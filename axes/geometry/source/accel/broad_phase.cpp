#include "ax/geometry/accel/broad_phase.hpp"

namespace ax::geo {

void BroadPhaseBase::ExpandAABB(real epsilon) {
  for (auto& c : colliders_) {
    c.aabb_.min().array() -= epsilon;
    c.aabb_.max().array() += epsilon;
  }
}

BroadPhaseResult const& BroadPhaseBase::GetCollidingPairs() const { return collidings_; }
}  // namespace ax::geo
