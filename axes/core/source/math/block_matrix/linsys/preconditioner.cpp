#include "ax/math/block_matrix/linsys/preconditioner.hpp"

namespace ax::math {

void BlockPreconditionerBase::SetProblem(std::shared_ptr<BlockedLinsysProblem> problem) {
  problem_ = problem;
}

}  // namespace ax::math