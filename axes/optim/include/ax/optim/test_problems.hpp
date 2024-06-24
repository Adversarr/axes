#pragma once
#include "common.hpp"
namespace ax::optim::test {

class RosenbrockProblem : public OptProblem {
public:
  RosenbrockProblem();

  math::vecxr Optimal(math::vecxr const& x0);
};

class LeastSquareProblem : public OptProblem {
public:
  LeastSquareProblem(math::matxxr const& A, math::vecxr const& b);

  math::vecxr Optimal(math::vecxr const& x0);

private:
  math::matxxr A_;
  math::vecxr b_;
};

class SparseLeastSquareProblem : public OptProblem {
public:
  SparseLeastSquareProblem(math::spmatr const& A, math::vecxr const& b);

  math::vecxr Optimal(math::vecxr const& x0);
private:
  math::spmatr A_;
  math::vecxr b_;
};

}  // namespace ax::optim::test
