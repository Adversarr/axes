#pragma once

#include "axes/optim/linesearch/linesearch.hpp"
#include "optimizer_base.hpp"

namespace ax::optim {

class Newton : public OptimizerBase {

public:
  /****************************** Ctor Dtor ******************************/
  explicit Newton(OptProblem& problem) : OptimizerBase{problem} {}

  ~Newton() override = default;

  /****************************** Interfaces ******************************/
  OptResult Optimize(math::vecxr const& x0, utils::Opt const& options) override;

  /****************************** Getter Setter ******************************/

protected:
};

}
