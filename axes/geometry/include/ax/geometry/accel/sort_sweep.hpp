#pragma once
#include "broad_phase.hpp"

namespace ax::geo {

class BroadPhase_SortSweep : public BroadPhaseBase {
public:
  BroadPhase_SortSweep() = default;
  ~BroadPhase_SortSweep() override = default;

  void DetectCollisions() override;
};

}
