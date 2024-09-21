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
  OptResult Optimize(OptProblem const& problem, Variable const& x) const override;

  OptimizerKind GetKind() const override { return OptimizerKind::Newton; }

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const override;

  /****************************** Getter Setter ******************************/
  std::unique_ptr<LinesearchBase> linesearch_;
  std::unique_ptr<math::DenseSolverBase> dense_solver_;
  std::unique_ptr<math::HostSparseSolverBase> sparse_solver_;
};

}  // namespace ax::optim
