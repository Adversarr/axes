#pragma once
#include "ax/optim/common.hpp"
#include "ax/utils/opt.hpp"
#include "linesearch.hpp"

namespace ax::optim {

class Linesearch_Wofle : public LinesearchBase {
public:
  Linesearch_Wofle() = default;
  ~Linesearch_Wofle() override = default;

  OptResult Optimize(OptProblem const& prob, Variable const& x0, Gradient const& grad,
                     Variable const& dir) const override;

  LineSearchKind GetKind() const override;
  void SetOptions(utils::Options const& options) override;
  utils::Options GetOptions() const override;

  // Typical value for many Gradient based methods, which is a common choice.
  // See Page 142, Numerical Optimization, Nocedal and Wright.
  Real step_shrink_rate_ = 0.5;         ///< Used in zoom function, see [Algorithm 3.6]
  Real step_expand_rate_ = 2.0;         ///< Used in outer iteration, see [Algorithm 3.5]
  Real required_descent_rate_ = 1e-4;   ///< c1: Armijo condition
  Real required_curvature_rate_ = 0.9;  ///< c2: Wolfe condition
};

}  // namespace ax::optim
