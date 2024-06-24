#pragma once

#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"
#include "ax/math/linsys/dense.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/utils/common.hpp"

namespace ax::optim {

class Optimizer_Newton : public OptimizerBase {
public:
  /****************************** Ctor Dtor ******************************/
  explicit Optimizer_Newton();

  ~Optimizer_Newton() override = default;

  /****************************** Interfaces ******************************/
  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) const override;

  OptimizerKind GetKind() const override { return OptimizerKind::kNewton; }

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const override;

  /****************************** Getter Setter ******************************/

protected:
  UPtr<LinesearchBase> linesearch_;
  UPtr<math::DenseSolverBase> dense_solver_;
  UPtr<math::SparseSolverBase> sparse_solver_;
};

}  // namespace ax::optim
