#pragma once

#include "ax/fem/timestep2/timestep.hpp"
#include "ax/math/sparse_matrix/linsys/solver/cg.hpp"

namespace ax::fem {

class TimeStep_QuasiNewton : public TimeStepBase {
public:
  explicit TimeStep_QuasiNewton(shared_not_null<Mesh> mesh);

  void SolveStep() override;

  void SetDensity(ConstRealBufferView density) override;

  void SetLame(ConstRealBufferView lame) override;

private:
  math::GeneralSparseSolver_ConjugateGradient solver_;
  Problem approximate_problem_;
};

}  // namespace ax::fem