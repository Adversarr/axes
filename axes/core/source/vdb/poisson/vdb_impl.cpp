#include "ax/vdb/poisson/vdb_impl.hpp"

#include <openvdb/tools/PoissonSolver.h>

namespace ax::vdb {

RealGridPtr VdbPoissonSolver::operator()(RealGridPtr source) {
  openvdb::math::pcg::State state;
  state.iterations = max_iterations_;
  state.relativeError = rel_error_;
  state.absoluteError = abs_error_;
  state.success = false;

  openvdb::util::NullInterrupter interrupter;
  RealTreePtr solution_tree;

  if (!boundary_condition_) {
    solution_tree = openvdb::tools::poisson::solve(source->tree(), state, interrupter);
  } else {
    solution_tree = openvdb::tools::poisson::solveWithBoundaryConditions(
        source->tree(), boundary_condition_, state, interrupter);
  }
  return RealGrid::create(solution_tree);
}

void VdbPoissonSolver::SetOptions(utils::Options const& option) {
  AX_SYNC_OPT_IF(option, idx, max_iterations) { AX_CHECK_GT(max_iterations_, 0); }
  AX_SYNC_OPT_IF(option, real, rel_error) { AX_CHECK_GT(rel_error_, 0); }
  AX_SYNC_OPT_IF(option, real, abs_error) { AX_CHECK_GT(abs_error_, 0); }
}

utils::Options VdbPoissonSolver::GetOptions() const {
  return utils::Options{
      {"max_iterations", max_iterations_},
      {"rel_error", rel_error_},
      {"abs_error", abs_error_},
  };
}

}  // namespace ax::vdb
