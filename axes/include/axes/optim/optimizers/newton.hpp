#pragma once

#include "axes/optim/linesearch/linesearch.hpp"
#include "axes/optim/optimizer_base.hpp"
#include "axes/math/linsys/dense.hpp"
#include "axes/math/linsys/sparse.hpp"
#include "axes/utils/common.hpp"

namespace ax::optim {

class Newton : public OptimizerBase {
public:
  /****************************** Ctor Dtor ******************************/
  explicit Newton();

  ~Newton() override = default;

  /****************************** Interfaces ******************************/
  OptResult Optimize(OptProblem const& problem, math::vecxr const& x0) override;

  void SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const override;

  /****************************** Getter Setter ******************************/

protected:
  std::string linesearch_name_;
  std::string dense_solver_name_;
  std::string sparse_solver_name_;
  utils::uptr<LineSearchBase> linesearch_;
  utils::uptr<math::DenseSolverBase> dense_solver_;
  utils::uptr<math::SparseSolverBase> sparse_solver_;
};

}  // namespace ax::optim
