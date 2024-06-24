#include "ax/math/linsys/preconditioner/Identity.hpp"

namespace ax::math {
void Preconditioner_Identity::AnalyzePattern() {}
void Preconditioner_Identity::Factorize() {}
vecxr Preconditioner_Identity::Solve(vecxr const &b) { return b; }
}  // namespace ax::math
