#pragma once
#include "ax/fem/timestepper.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace ax::fem {

template <idx dim> class Timestepper_NaiveOptim : public TimeStepperBase<dim> {
public:
  Timestepper_NaiveOptim();
  Timestepper_NaiveOptim(SPtr<TriMesh<dim>> mesh);
  virtual ~Timestepper_NaiveOptim() = default;
  void SolveTimestep() final;

  void SetOptions(utils::Options const& opt) final;
  utils::Options GetOptions() const final;

private:
  UPtr<optim::OptimizerBase> optimizer_;
};

}  // namespace ax::fem
