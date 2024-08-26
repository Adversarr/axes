#pragma once
#include "ax/optim/common.hpp"

namespace ax::optim::test {

class RosenbrockProblem : public OptProblem {
public:
  RosenbrockProblem();

  Variable Optimal(Variable const& x0);
};

class LeastSquareProblem : public OptProblem {
public:
  LeastSquareProblem(DenseHessian const& A, Variable const& b);

  Variable Optimal(Variable const& x0);

private:
  DenseHessian A_;
  Variable b_;
};

class SparseLeastSquareProblem : public OptProblem {
public:
  SparseLeastSquareProblem(SparseHessian const& A, Variable const& b);

  Variable Optimal(Variable const& x0);

private:
  SparseHessian A_;
  Variable b_;
};

class PoissonProblem : public SparseLeastSquareProblem {
public:
  explicit PoissonProblem(Index n);
};

}  // namespace ax::optim::test
