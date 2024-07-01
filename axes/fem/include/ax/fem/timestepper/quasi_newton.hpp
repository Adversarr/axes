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

  void SetOptions(const utils::Options& option) override;
  utils::Options GetOptions() const override;

  optim::Optimizer_Lbfgs& GetOptimizer() { return optimizer_; }

  math::spmatr GetLaplacianAsApproximation() const;

protected:
  real dt_back_ = -1;
  SPtr<math::SparseSolverBase> solver_;
  LbfgsStrategy strategy_ = LbfgsStrategy::kNaive;
  optim::Optimizer_Lbfgs optimizer_;

  math::spmatr static_inverse_approximation_;
};

struct SparseInverseApproximator {
  math::spmatr A_;
  real eig_modification_{0.};
  bool require_check_secant_{false};
};

}  // namespace ax::fem
