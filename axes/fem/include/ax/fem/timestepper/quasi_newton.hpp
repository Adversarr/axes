#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/math/linsys/sparse.hpp"
namespace ax::fem {

BOOST_DEFINE_ENUM_CLASS(LbfgsStrategy,
  kNaive,
  kLaplacian,
  kHard);

template<idx dim>
class Timestepper_QuasiNewton : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~Timestepper_QuasiNewton() = default;

  virtual Status Initialize() final;
  void SetLbfgsStrategy(LbfgsStrategy strategy) { strategy_ = strategy; }
  void UpdateSolverLaplace();

  // Override the defaults.
  void BeginSimulation(real dt) final;
  void BeginTimestep(real dt) final;
  void SolveTimestep() final;

protected:
  real dt_back_ = -1;

  UPtr<math::SparseSolverBase> solver_;
  LbfgsStrategy strategy_;
};

}
