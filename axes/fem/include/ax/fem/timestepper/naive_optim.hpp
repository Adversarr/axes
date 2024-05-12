#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::fem {

template <idx dim> class Timestepper_NaiveOptim : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~Timestepper_NaiveOptim() = default;
  void SolveTimestep() final;
};

}  // namespace ax::fem
