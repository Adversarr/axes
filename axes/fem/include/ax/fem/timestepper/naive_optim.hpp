#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::fem {

template <int dim> class Timestepper_NaiveOptim : public TimeStepperBase<dim> {
public:
  Timestepper_NaiveOptim(std::shared_ptr<TriMesh<dim>> mesh);
  virtual ~Timestepper_NaiveOptim() = default;
  void SolveTimestep() final;

  void SetOptions(utils::Options const& opt) final;
  utils::Options GetOptions() const final;

private:
  std::unique_ptr<optim::OptimizerBase> optimizer_;
};

}  // namespace ax::fem
