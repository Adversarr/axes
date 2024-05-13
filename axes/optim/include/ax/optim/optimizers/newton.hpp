#pragma once

#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"
#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/utils/common.hpp"

namespace ax::optim {

class Newton : public OptimizerBase {
public:
  /****************************** Ctor Dtor ******************************/
  explicit Newton();

  ~Newton() override = default;

  /****************************** Interfaces ******************************/
  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  OptimizerKind GetKind() const override { return OptimizerKind::kNewton; }

  void SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const override;

  /****************************** Getter Setter ******************************/

protected:
  std::string linesearch_name_;
  std::string dense_solver_name_;
  std::string sparse_solver_name_;
  UPtr<LinesearchBase> linesearch_;
  UPtr<math::DenseSolverBase> dense_solver_;
  UPtr<math::SparseSolverBase> sparse_solver_;
};

}  // namespace ax::optim
