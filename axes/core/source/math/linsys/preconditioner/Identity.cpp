#include "ax/math/linsys/preconditioner/Identity.hpp"

namespace ax::math {
void Preconditioner_Identity::AnalyzePattern() {}
void Preconditioner_Identity::Factorize() {}
matxxr Preconditioner_Identity::Solve(matxxr const &b) { return b; }
}  // namespace ax::math
