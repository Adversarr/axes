#include "axes/optim/newton.hpp"
#include "axes/math/functional.hpp"
#include "axes/core/echo.hpp"
#include "axes/core/init.hpp"

using namespace ax;

real rosenbrock(math::vecxr const& x) {
  real f = 0;
  f = (1 - x(0)) * (1 - x(0)) + 100 * (x(1) - x(0) * x(0)) * (x(1) - x(0) * x(0));
  return f;
}

math::vecxr rosenbrock_grad(math::vecxr const& x) {
  math::vecxr grad(2);
  grad(0) = -2 * (1 - x(0)) - 400 * x(0) * (x(1) - x(0) * x(0));
  grad(1) = 200 * (x(1) - x(0) * x(0));
  return grad;
}

math::matxxr rosenbrock_hessian(math::vecxr const& x) {
  math::matxxr hessian = math::zeros(2, 2);
  hessian(0, 0) = 2 - 400 * x(1) + 1200 * x(0) * x(0);
  hessian(0, 1) = -400 * x(0);
  hessian(1, 0) = -400 * x(0);
  hessian(1, 1) = 200;

  math::matxxr projected = hessian;
  for (idx i = 0; i < 2; ++i) {
    real abs_sum_off_diag = 0;
    for (idx j = 0; j < 2; ++j) {
      if (i != j) {
        abs_sum_off_diag += std::abs(hessian(i, j));
      }
    }
    projected(i, i) += abs_sum_off_diag;
  }
  return projected;
}

void verbose(idx iter, math::vecxr const &x, real f) {
  LOG(INFO) << "iter: " << iter << ", x: " << x.transpose() << ", f: " << f;
}

int main(int argc, char** argv) {
  init(argc, argv);
  optim::OptProblem prob;

  prob.SetEnergy(rosenbrock)
      .SetGrad(rosenbrock_grad)
      .SetHessian(rosenbrock_hessian)
      .SetVerbose(verbose);

  optim::Newton newton{prob};
  math::vecxr x0 = math::vecxr::Ones(2) * 2;
  auto solution = newton.Optimize(x0, {});
  clean_up();
  return 0;
}