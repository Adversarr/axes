#include "ax/vdb/poisson.hpp"

#include "ax/vdb/poisson/vdb_impl.hpp"
// #include "ax/vdb/poisson/multigrid_impl.hpp"

namespace ax::vdb {

void PoissonSolverBase::SetBoundaryCondition(BcFunc boundary_condition) {
  boundary_condition_ = boundary_condition;
}

UPtr<PoissonSolverBase> PoissonSolverBase::Create(PoissonSolverKind kind) {
  switch (kind) {
    case PoissonSolverKind::kVdb:
      return std::make_unique<VdbPoissonSolver>();
      // case PoissonSolverKind::kMultigrid:
      //   return utils::make_unique<MultigridPoissonSolver>();
  }
  AX_UNREACHABLE();
}

}  // namespace ax::vdb