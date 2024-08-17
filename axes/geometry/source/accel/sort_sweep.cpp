#include "ax/geometry/accel/sort_sweep.hpp"

namespace ax::geo {

void BroadPhase_SortSweep::DetectCollisions() {
  auto& colliders = this->colliders_;
  // sort it in x
  std::sort(colliders.begin(), colliders.end(),
            [](auto const& a, auto const& b) { return a.aabb_.min().x() < b.aabb_.min().x(); });

  ClearCollidingPairs();

  // sweep
  for (Index i = 0; i < (Index)colliders.size(); ++i) {
    for (Index j = i + 1; j < (Index)colliders.size(); ++j) {
      if (colliders[j].aabb_.min().x() > colliders[i].aabb_.max().x()) {
        break;
      }

      if (has_collide<3>(colliders[i].aabb_, colliders[j].aabb_)) {
        AddCollidingPair(i, j);
      }
    }
  }
}

}  // namespace ax::geo
