#pragma once
#include "broad_phase.hpp"

namespace ax::geo {

class BroadPhase_FlatOctree : public BroadPhaseBase {
public:
  struct Impl;
  void DetectCollisions() override;
  BroadPhase_FlatOctree();

  ~BroadPhase_FlatOctree();

  void ForeachTreeAABB(std::function<void(AlignedBox3 const&)> f) const;

private:
  UPtr<Impl> impl_;
};

}