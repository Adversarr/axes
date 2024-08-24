#pragma once
#include "linesearch.hpp"

namespace ax::optim {

class Linesearch_Backtracking : public LinesearchBase {
public:
  Linesearch_Backtracking() = default;

  OptResult Optimize(OptProblem const& prob, Variable const& x0, Gradient const& grad,
                     Variable const& dir) const override;

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const final;

  LineSearchKind GetKind() const override;

  Real step_shrink_rate_ = 0.5;
  Real required_descent_rate_ = 1e-4;
};

}  // namespace ax::optim
