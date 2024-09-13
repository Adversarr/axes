#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/optim/optimizers/pcg.hpp"

namespace ax::fem {

template <int dim>
class Timestepper_NonlinearCg final : public TimeStepperBase<dim> {
public:
  explicit Timestepper_NonlinearCg(std::shared_ptr<LinearMesh<dim>> mesh);
  ~Timestepper_NonlinearCg() override;

  void SolveTimestep() override;

  void SetOptions(utils::Options const& opt) override;

  virtual void BeginSimulation(Real dt);

  utils::Options GetOptions() const override;
  optim::Optimizer_NonlinearCg* GetOptimizer();

private:
  std::unique_ptr<optim::Optimizer_NonlinearCg> optimizer_;

  optim::Variable DoPreconditioning(optim::Variable const& x0, optim::Gradient const& gradient);

  math::RealVectorX jacob_;
  math::RealMatrixX basis_;
};

}  // namespace ax::fem