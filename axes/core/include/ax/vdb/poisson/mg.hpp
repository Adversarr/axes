#pragma once
#include "ax/vdb/poisson.hpp"

namespace ax::vdb {

///< Poisson solver using multigrid method with Jacobi smoother
class MultiGridPoissonSolver : public PoissonSolverBase {
public:
  StatusOr<RealGridPtr> operator()(RealGridPtr source) final;
};

}