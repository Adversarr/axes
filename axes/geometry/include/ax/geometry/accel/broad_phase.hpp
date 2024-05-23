#pragma once
#include "ax/geometry/hittables.hpp"
namespace ax::geo {

class BroadPhaseBase {
public:
  /************************* SECT: interface *************************/
  virtual ~BroadPhaseBase() = default;

  virtual void DetectCollisions() = 0;
  virtual void ClearCollidingPairs() { collidings_.clear(); }

  void Clear() { colliders_.clear(); }
  ColliderInfo const& GetCollider(idx i) const { return colliders_[i]; }
  BroadPhaseResult const& GetCollidingPairs() const;
  void ExpandAABB(real epsilon);

  /************************* SECT: Add Collider *************************/
  AX_FORCE_INLINE void AddCollider(AlignedBox3 const& aabb, idx external_id, idx parent_id,
                                   PrimitiveKind external_kind) {
    colliders_.push_back({.aabb_ = aabb,
                          .external_id_ = external_id,
                          .parent_id_ = parent_id,
                          .external_kind_ = external_kind});
  }

protected:
  std::vector<ColliderInfo> colliders_;
  BroadPhaseResult collidings_;
};

}  // namespace ax::geo
