#include "ax/math/linsys/preconditioner/Identity.hpp"

#include "ax/utils/status.hpp"

namespace ax::math {
Status PreconditionerIdentity::Analyse(LinsysProblem_Sparse const &) { AX_RETURN_OK(); }

vecxr PreconditionerIdentity::Solve(vecxr const &b, vecxr const &) { return b; }
}  // namespace ax::math