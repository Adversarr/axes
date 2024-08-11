#pragma once
#include "common.hpp"
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

}  // namespace ax::optim::test
