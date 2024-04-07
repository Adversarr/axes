#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/math/linsys/sparse.hpp"
namespace ax::fem {

template<idx dim>
class Timestepper_QuasiNewton : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~Timestepper_QuasiNewton() = default;
  virtual Status Init(utils::Opt const &opt = {}) final;
  virtual Status Step(real dt = 0.01) final;

protected:
  UPtr<math::SparseSolverBase> solver_;
};

}