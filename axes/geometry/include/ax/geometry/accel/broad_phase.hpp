#pragma once
#include "ax/geometry/hittables.hpp"
#include "ax/utils/opt.hpp"
namespace ax::geo {

class BroadPhaseBase : utils::Tunable {
public:
  /************************* SECT: interface *************************/
  virtual ~BroadPhaseBase() = default;

  virtual void DetectCollisions() = 0;
  virtual void ClearCollidingPairs() { collidings_.clear(); }

  void Clear() { colliders_.clear(); }
  ColliderInfo const& GetCollider(Index i) const { return colliders_[i]; }
  BroadPhaseResult const& GetCollidingPairs() const;
  void ExpandAABB(Real epsilon);
  void Reserve(size_t n);

  /************************* SECT: Add Collider *************************/
  AX_FORCE_INLINE Index AddCollider(AlignedBox3 const& aabb, Index external_id, Index parent_id,
                                  PrimitiveKind external_kind) {
    colliders_.push_back({.aabb_ = aabb,
                          .external_id_ = external_id,
                          .parent_id_ = parent_id,
                          .external_kind_ = external_kind});
    return static_cast<Index>(colliders_.size() - 1);
  }


protected:
  void AddCollidingPair(Index a, Index b);
  std::vector<ColliderInfo> colliders_;
  BroadPhaseResult collidings_;
};

}  // namespace ax::geo
