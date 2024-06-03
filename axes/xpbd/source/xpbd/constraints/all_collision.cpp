#include "ax/xpbd/constraints/all_collision.hpp"

namespace ax::xpbd {

// two main collision:
// 1. vertex-to-triangle
// 2. edge-to-edge
// for each collision, we need to determine the real collision type:
// 1. v-t
// 2. v-e
// 3. v-v
// 3. e-e
// for each type of collision, we need to define the `relax` method.

ConstraintSolution Constraint_AllCollision::SolveDistributed() {
  // TODO: Implement this method.
}

void Constraint_AllCollision::BeginStep() {
}

void Constraint_AllCollision::EndStep() {}


void Constraint_AllCollision::UpdatePositionConsensus() {
  
}


}