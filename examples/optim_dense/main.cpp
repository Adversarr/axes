#include <absl/flags/flag.h>

#include "axes/core/echo.hpp"
#include "axes/core/init.hpp"
#include "axes/optim/optimizers/lbfgs.hpp"
#include "axes/optim/optimizers/newton.hpp"
#include "axes/optim/test_problems.hpp"

using namespace ax;
idx n = 3;
math::matxxr A;
math::vecxr b;

ABSL_FLAG(std::string, problem, "sp_lstsq", "Problem to solve");
ABSL_FLAG(std::string, optimizer, "lbfgs", "Optimizer");

int main(int argc, char** argv) {
  init(argc, argv);

  A = math::matxxr::Random(n, n);
  A = (math::eye(n) + A * A.transpose()).eval();
  b = math::vecxr::Random(n);

  optim::OptProblem* prob;
  math::vecxr optimal;
  math::vecxr x0;

  utils::Opt opt{
      {"verbose", 1},
  };

  if (absl::GetFlag(FLAGS_problem) == "rosenbrock") {
    prob = new optim::test::RosenbrockProblem();
    optimal = optim::test::RosenbrockProblem{}.Optimal(b);
    x0 = math::ones(n, 1);
    x0 = ((x0.array() + 1) * 0.3 + 1).matrix().eval();
  } else if (absl::GetFlag(FLAGS_problem) == "lstsq") {
    prob = new optim::test::LeastSquareProblem(A, b);
    optimal = optim::test::LeastSquareProblem{A, b}.Optimal(b);
    x0.setRandom(n);
  } else if (absl::GetFlag(FLAGS_problem) == "sp_lstsq") {
    math::SparseCoeffVec A_sparse;
    A_sparse.reserve(n * 10);
    for (idx i = 0; i < n; ++i) {
      for (idx j = 0; j < 9; ++j) {
        A_sparse.push_back({i, rand() % n, real(rand() % 100) / 100.0});
      }
      A_sparse.push_back({i, i, real(1.0)});
    }
    math::sp_matxxr A2{n, n};
    b.setRandom(n);
    A2.setFromTriplets(A_sparse.begin(), A_sparse.end());
    A2.makeCompressed();
    A2 = A2.transpose() * A2;
    prob = new optim::test::SparseLeastSquareProblem(A2, b);
    optimal = b;
    x0 = math::ones(n, 1) * 1.2;
  } else {
    LOG(FATAL) << "Unknown problem: " << absl::GetFlag(FLAGS_problem);
  }
  optim::OptimizerBase* optimizer;
  if (absl::GetFlag(FLAGS_optimizer) == "newton") {
    optimizer = new optim::Newton(*prob);
  } else if (absl::GetFlag(FLAGS_optimizer) == "lbfgs") {
    optimizer = new optim::Lbfgs(*prob);
  } else {
    LOG(FATAL) << "Unknown optimizer: " << absl::GetFlag(FLAGS_optimizer);
  }

  auto solution = optimizer->Optimize(x0, opt);

  CHECK(solution->converged_);
  LOG(INFO) << "Solution: " << solution->x_opt_.transpose();
  LOG(INFO) << "Accurate: " << optimal.transpose();

  delete prob;
  clean_up();
  return 0;
}
