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
  math::matxxr A;
  math::vecxr b;
};
}  // namespace ax::optim::test
