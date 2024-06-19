#include "ax/math/linsys/preconditioner/Identity.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {
void Preconditioner_Identity::Analyse(LinsysProblem_Sparse const &) {}

vecxr Preconditioner_Identity::Solve(vecxr const &b, vecxr const &) { return b; }
}  // namespace ax::math