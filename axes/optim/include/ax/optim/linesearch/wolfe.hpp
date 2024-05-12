#pragma once
#include "ax/optim/common.hpp"
#include "ax/utils/common.hpp"
#include "ax/utils/opt.hpp"
#include "linesearch.hpp"
namespace ax::optim {


class Linesearch_Wofle : public LinesearchBase {
public:
  virtual ~Linesearch_Wofle() = default;
  virtual OptResult Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& grad, math::vecxr const& dir) const;
  virtual LineSearchKind GetKind() const;
  virtual Status SetOptions(utils::Opt const& options);
  virtual utils::Opt GetOptions() const;



  // Typical value for Newton's method:
  real initial_step_length_ = 1.0;
  real step_shrink_rate_ = 0.5;
  real required_descent_rate_ = 1e-4;
  real required_curvature_rate_ = 0.9;

  bool strong_wolfe_ = false;
};

}  // namespace ax::optim
