#include "axes/optim/test_problems.hpp"

#include "axes/core/echo.hpp"
#include "axes/math/functional.hpp"
#include "axes/math/linsys/dense.hpp"
#include "axes/optim/spsdm/eigenvalue.hpp"

namespace ax::optim::test {

/************************* SECT: RosenbrockProblem *************************/
real rosenbrock(math::vecxr const& x) {
  real f = 0;
  for (idx i = 0; i < x.size() - 1; ++i) {
    f += math::square(x[i] - 1)
         + 100 * math::square(x[i + 1] - math::square(x[i]));
  }
  return f;
}

math::vecxr rosenbrock_grad(math::vecxr const& x) {
  math::vecxr grad(x.size());
  for (idx i = 0; i < x.size() - 1; ++i) {
    grad[i] = -2 * (1 - x[i]) - 400 * x[i] * (x[i + 1] - math::square(x[i]));
  }
  grad[x.size() - 1] = 200 * (x[x.size() - 1] - math::square(x[x.size() - 2]));
  return grad;
}

math::matxxr rosenbrock_hessian(math::vecxr const& x) {
  math::matxxr hessian = math::zeros(x.size(), x.size());
  for (idx i = 0; i < x.size() - 1; ++i) {
    hessian(i, i) = 2 - 400 * x[i + 1] + 1200 * math::square(x[i]);
    hessian(i, i + 1) = -400 * x[i];
    hessian(i + 1, i) = -400 * x[i];
  }
  hessian(x.size() - 1, x.size() - 1) = 200;

  optim::EigenvalueModification modification;
  auto spsd_hessian = modification.Modify(hessian);
  CHECK_OK(spsd_hessian);
  return spsd_hessian.value();
}

math::vecxr RosenbrockProblem::Optimal(math::vecxr const& x0) {
  return math::vecxr::Ones(x0.size());
}

RosenbrockProblem::RosenbrockProblem() {
  SetEnergy(rosenbrock);
  SetGrad(rosenbrock_grad);
  SetHessian(rosenbrock_hessian);
}

/************************* SECT: Least Square *************************/
LeastSquareProblem::LeastSquareProblem(math::matxxr const& A,
                                       math::vecxr const& b)
    : A(A), b(b) {
  SetEnergy([&](math::vecxr const& x) {
    return 0.5 * (x - b).transpose() * A * (x - b);
  });
  SetGrad([&](math::vecxr const& x) { return A * (x - b); });
  SetHessian([&](math::vecxr const&) { return A; });
}

math::vecxr LeastSquareProblem::Optimal(math::vecxr const&) { return b; }

}  // namespace ax::optim::test
