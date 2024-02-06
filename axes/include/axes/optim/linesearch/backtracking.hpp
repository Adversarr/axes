#pragma once
#include "linesearch.hpp"
#include "axes/utils/status.hpp"

namespace ax::optim {

class BacktrackingLinesearch : public LineSearchBase {
public:
  BacktrackingLinesearch() {}

  OptResult Optimize(OptProblem const& prob,math::vecxr const& x0, math::vecxr const& dir, utils::Opt const& options) override;

  real alpha_ = 1.0;
  real rho_ = 0.3;
  real c_ = 0.3;
};

}