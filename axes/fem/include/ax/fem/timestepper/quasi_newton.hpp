#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/math/linsys/sparse.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
namespace ax::fem {

AX_DEFINE_ENUM_CLASS(LbfgsStrategy, kNaive, kLaplacian, kHard, kReservedForExperimental);

template <int dim> class Timestepper_QuasiNewton : public TimeStepperBase<dim> {
public:
  using TimeStepperBase<dim>::TimeStepperBase;
  virtual ~Timestepper_QuasiNewton() = default;

  virtual void Initialize() final;
  void SetLbfgsStrategy(LbfgsStrategy strategy) { strategy_ = strategy; }
  void UpdateSolverLaplace();

  // Override the defaults.
  void BeginSimulation(Real dt) final;
  void BeginTimestep() final;
  void SolveTimestep() final;

  void SetOptions(const utils::Options& option) override;
  utils::Options GetOptions() const override;

  optim::Optimizer_Lbfgs& GetOptimizer() { return optimizer_; }

  math::RealSparseMatrix GetLaplacianAsApproximation() const;

protected:
  std::shared_ptr<math::HostSparseSolverBase> solver_;
  LbfgsStrategy strategy_ = LbfgsStrategy::kNaive;
  optim::Optimizer_Lbfgs optimizer_;

  math::RealSparseMatrix static_inverse_approximation_;
};

struct SparseInverseApproximator {
  math::RealSparseMatrix A_;
  math::RealVectorX precond_;
  Real eig_modification_{0.};
  bool require_check_secant_{false};
};

}  // namespace ax::fem
