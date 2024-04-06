#pragma once
#include "ax/fem/timestepper.hpp"

namespace ax::fem {

template<idx dim>
class Timestepper_NaiveOptim : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~Timestepper_NaiveOptim() = default;
  virtual Status Step(real dt = 0.01);
};

}