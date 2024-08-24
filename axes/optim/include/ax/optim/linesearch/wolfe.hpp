#pragma once
#include "ax/optim/common.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/opt.hpp"
#include "linesearch.hpp"
namespace ax::optim {

class Linesearch_Wofle : public LinesearchBase {
public:
  ~Linesearch_Wofle() override = default;
  OptResult Optimize(OptProblem const& prob, Variable const& x0, Gradient const& grad,
                     Variable const& dir) const override;
  LineSearchKind GetKind() const override;
  void SetOptions(utils::Options const& options) override;
  utils::Options GetOptions() const override;

  // Typical value for Newton's method:
  Real step_shrink_rate_ = 0.5;
  Real required_descent_rate_ = 1e-4;
  Real required_curvature_rate_ = 0.9;

  bool strong_wolfe_ = false;
};

}  // namespace ax::optim
