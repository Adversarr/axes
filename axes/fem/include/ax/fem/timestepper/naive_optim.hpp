#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::fem {

template <idx dim> class Timestepper_NaiveOptim : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~Timestepper_NaiveOptim() = default;
  virtual Status Step(real dt = 0.01);

  void SetOptimizer(UPtr<optim::OptimizerBase> optimizer) { optimizer_ = std::move(optimizer); }

private:
  UPtr<optim::OptimizerBase> optimizer_;
};

}  // namespace ax::fem
