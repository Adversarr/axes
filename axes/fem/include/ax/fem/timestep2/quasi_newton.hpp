#pragma once

#include "ax/fem/timestep2/timestep.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"

namespace ax::fem {

class TimeStep_QuasiNewton : public TimeStepBase {
public:
  explicit TimeStep_QuasiNewton(shared_not_null<Mesh> mesh);

  void Compute() override;

  void SolveStep() override;

  void SetDensity(ConstRealBufferView density) override;

  void SetLame(ConstRealBufferView lame) override;

  void SetTimeStep(Real dt) override;

private:
  std::unique_ptr<math::GeneralSparseSolverBase> solver_;
  std::unique_ptr<Problem> approximate_problem_;
  BufferPtr<Real> temp_;
};

}  // namespace ax::fem