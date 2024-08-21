#include "ax/math/linsys/preconditioner/Identity.hpp"

namespace ax::math {
void Preconditioner_Identity::AnalyzePattern() {}
void Preconditioner_Identity::Factorize() {}
RealMatrixX Preconditioner_Identity::Solve(RealMatrixX const &b) { return b; }
}  // namespace ax::math
