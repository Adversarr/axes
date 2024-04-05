#pragma once
#include "ax/fem/timestepper.hpp"

namespace ax::fem {

template<idx dim>
class TimeStepperNewton : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~TimeStepperNewton() = default;
  virtual Status Step(real dt = 0.01);
};

}