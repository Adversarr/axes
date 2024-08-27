#pragma once
#include "linesearch.hpp"

namespace ax::optim {

// Implements Algorithm 3.1 in Nocedal and Wright
class Linesearch_Backtracking : public LinesearchBase {
public:
  Linesearch_Backtracking() = default;

  OptResult Optimize(OptProblem const& prob, Variable const& x0, Gradient const& grad,
                     Variable const& dir) const override;

  void SetOptions(utils::Options const& options) override;

  utils::Options GetOptions() const final;

  LineSearchKind GetKind() const override;

  Real step_shrink_rate_ = 0.;         // rho: rate to shrink step size, default is adaptive, see below.
  Real required_descent_rate_ = 1e-4;  // c1:  armijo condition

  // The following two parameters are used to implement the adaptive step size, if step_shrink_rate_
  // is set to zero or negative, the step size will be adaptively adjusted based on the following
  // two parameters. See the comment below [Algorithm 3.1] (using a quadratic model.)
  Real min_step_shrink_rate_ = 0.25;  // rho_lo: minimum step size
  Real max_step_shrink_rate_ = 0.75;  // rho_hi: maximum step size
};

}  // namespace ax::optim
