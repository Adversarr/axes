#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/init.hpp"
#include "axes/optim/optimizers/lbfgs.hpp"
#include "axes/optim/optimizers/newton.hpp"
#include "axes/optim/test_problems.hpp"

using namespace ax;
idx n = 2;
math::matxxr A;
math::vecxr b;

ABSL_FLAG(std::string, problem, "least_square", "Problem to solve");

int main(int argc, char** argv) {
  init(argc, argv);

  A = math::matxxr::Random(n, n);
  A = (math::eye(n) + A * A.transpose()).eval();
  b = math::vecxr::Random(n);

  optim::OptProblem prob;
  math::vecxr optimal;
  math::vecxr x0;

  utils::Opt opt {
    { "verbose", 1 },
  };

  if (absl::GetFlag(FLAGS_problem) == "rosenbrock") {
    prob = optim::test::RosenbrockProblem();
    optimal = optim::test::RosenbrockProblem{}.Optimal(b);
    x0.setRandom(n);
    x0 = ((x0.array() + 1) * 0.3 + 1).matrix().eval();
  } else if (absl::GetFlag(FLAGS_problem) == "least_square") {
    prob = optim::test::LeastSquareProblem(A, b);
    optimal = optim::test::LeastSquareProblem{A, b}.Optimal(b);
    x0 = math::ones(n, 1) * 1.2;
  }
  optim::Newton optimizer{prob};
  auto solution = optimizer.Optimize(x0, opt);

  CHECK(solution->converged_);
  LOG(INFO) << "Solution: " << solution->x_opt_.transpose();
  LOG(INFO) << "Accurate: " << optimal.transpose();
  clean_up();
  return 0;
}
