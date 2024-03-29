#pragma once
#include "axes/vdb/poisson.hpp"

namespace ax::vdb {

class JacobiPoissonSolver : public PoissonSolverBase {
public:
  StatusOr<RealGridPtr> operator()(RealGridPtr source) final;
};

}