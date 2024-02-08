#pragma once
#include "linesearch.hpp"
#include "axes/utils/status.hpp"

namespace ax::optim {

class BacktrackingLinesearch : public LinesearchBase {
public:
  BacktrackingLinesearch() {}

  OptResult Optimize(OptProblem const& prob,math::vecxr const& x0, math::vecxr const& dir) const override;

  Status SetOptions(utils::Opt const& options) override;

  utils::Opt GetOptions() const final;

  real alpha_ = 1.0;
  real rho_ = 0.5;
  real c_ = 1e-4;
};

}
