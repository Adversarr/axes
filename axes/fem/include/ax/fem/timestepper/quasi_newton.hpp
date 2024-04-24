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
  virtual Status Init(utils::Opt const &opt = {}) final;
  virtual Status Step(real dt = 0.01) final;

  void SetLbfgsStrategy(LbfgsStrategy strategy) { strategy_ = strategy; }

protected:
  UPtr<math::SparseSolverBase> solver_;

  LbfgsStrategy strategy_;
};

}
