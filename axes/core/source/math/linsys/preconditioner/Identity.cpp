#include "ax/math/linsys/preconditioner/Identity.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {
void PreconditionerIdentity::Analyse(LinsysProblem_Sparse const &) {}

vecxr PreconditionerIdentity::Solve(vecxr const &b, vecxr const &) { return b; }
}  // namespace ax::math