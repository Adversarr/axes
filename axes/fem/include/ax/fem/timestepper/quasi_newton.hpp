#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
namespace ax::fem {

BOOST_DEFINE_ENUM_CLASS(LbfgsStrategy, kNaive, kLaplacian, kHard, kReservedForExperimental);

template <idx dim> class Timestepper_QuasiNewton : public TimeStepperBase<dim> {
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

  void SetOptions(const utils::Opt& option) override;
  utils::Opt GetOptions() const override;

  optim::Lbfgs& GetOptimizer() { return optimizer_; }

protected:
  real dt_back_ = -1;
  SPtr<math::SparseSolverBase> solver_;
  LbfgsStrategy strategy_ = LbfgsStrategy::kNaive;
  optim::Lbfgs optimizer_;
};

}  // namespace ax::fem
