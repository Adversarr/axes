#pragma once
#include "ax/math/linsys/sparse.hpp"

namespace ax::math {

class SparseSolver_ConjugateGradient : public SparseSolverBase {
public:
  Status Analyse(LinsysProblem_Sparse const &problem) override;

  result_type Solve(vecxr const &b, vecxr const &x0) override;

  Status SetOptions(utils::Opt const &) final;

  utils::Opt GetOptions() const final;

  SparseSolverKind Kind() const final { return SparseSolverKind::kConjugateGradient; }

private:
  Eigen::ConjugateGradient<sp_matxxr, Eigen::Lower | Eigen::Upper,
                           Eigen::IncompleteCholesky<real, Eigen::Lower, Eigen::AMDOrdering<idx>>>
      solver_;

  LinsysProblem_Sparse sparse_problem_;

  idx max_iter_ = 100;
  real tol_ = 1e-6;
};
}  // namespace ax::math
