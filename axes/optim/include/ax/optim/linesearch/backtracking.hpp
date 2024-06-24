#pragma once
#include "linesearch.hpp"
#include "ax/utils/status.hpp"

namespace ax::optim {

class Linesearch_Backtracking : public LinesearchBase {
public:
  Linesearch_Backtracking() {}

  OptResult Optimize(OptProblem const& prob,math::vecxr const& x0, math::vecxr const& grad, math::vecxr const& dir) const override;

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const final;

  LineSearchKind GetKind() const override;

  real initial_step_length_ = 1.0;
  real step_shrink_rate_ = 0.5;
  real required_descent_rate_ = 1e-4;
};

}
