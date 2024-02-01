#pragma once
#include "axes/utils/common.hpp"
#include "linesearch.hpp"
namespace ax::optim {

class WolfeLineSearch : public LineSearchBase {
public:
  WolfeLineSearch(OptProblem& problem) : LineSearchBase{problem} {}

  OptResult Optimize(math::vecxr const& x0, math::vecxr const& dir) override;

  utils::DoNotUse<WolfeLineSearch>;

  real c1_{1e-4};
  real c2_{0.9};
};

}  // namespace ax::optim
