#include <absl/flags/flag.h>

#include "ax/core/echo.hpp"
#include "ax/core/init.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/test_problems.hpp"
#include "ax/utils/time.hpp"

using namespace ax;
idx n = 2;
math::matxxr A;
math::vecxr b;

ABSL_FLAG(std::string, problem, "sp_lstsq", "Problem to solve");
ABSL_FLAG(std::string, optimizer, "lbfgs", "Optimizer");
ABSL_FLAG(bool, verbose, false, "Verbose output");
ABSL_FLAG(idx, dof, 2, "Degrees of freedom");

int main(int argc, char** argv) {
  init(argc, argv);

  optim::OptimizerBase* optimizer;
  optim::OptProblem* prob;
  math::vecxr optimal;
  math::vecxr x0;
  n = absl::GetFlag(FLAGS_dof);

  /************************* SECT: Optimizer Options *************************/
  utils::Opt opt{
    {"verbose", idx(absl::GetFlag(FLAGS_verbose))},
    {"max_iter", idx(200)},
    {"linesearch_name", "kBacktracking"},
    {"linesearch_opt", utils::Opt{
      {"required_descent_rate", real(1e-4)},
    }}
  };

  /************************* SECT: Setup Problems *************************/
  if (absl::GetFlag(FLAGS_problem) == "rosenbrock") {
    prob = new optim::test::RosenbrockProblem();
    x0 = math::vecxr::Constant(n, 1.2);
    optimal = optim::test::RosenbrockProblem{}.Optimal(x0);
  } else if (absl::GetFlag(FLAGS_problem) == "lstsq") {
    A = math::matxxr::Random(n, n);
    A = (math::eye(n) + A * A.transpose()).eval();
    b = math::vecxr::Random(n);
    prob = new optim::test::LeastSquareProblem(A, b);
    optimal = optim::test::LeastSquareProblem{A, b}.Optimal(b);
    x0.setRandom(n);
  } else if (absl::GetFlag(FLAGS_problem) == "sp_lstsq") {
    math::sp_coeff_list A_sparse;
    A_sparse.reserve(n * 10);
    for (idx i = 0; i < n; ++i) {
      for (idx j = 0; j < 9; ++j) {
        A_sparse.push_back({i, rand() % n, real(rand() % 100) / 100.0});
      }
      A_sparse.push_back({i, i, real(10.0)});
    }
    math::sp_matxxr A2{n, n};
    b.setRandom(n);
    A2.setFromTriplets(A_sparse.begin(), A_sparse.end());
    A2.makeCompressed();
    A2 = A2.transpose() * A2;
    auto* sp_lstsq = new optim::test::SparseLeastSquareProblem(A2, b);
    prob = sp_lstsq;
    x0 = math::ones(n, 1) * 1.2;
    optimal = sp_lstsq->Optimal(x0);
  } else {
    AX_LOG(FATAL) << "Unknown problem: " << absl::GetFlag(FLAGS_problem);
  }

  /************************* SECT: Create Optimizer *************************/
  if (absl::GetFlag(FLAGS_optimizer) == "newton") {
    optimizer = new optim::Newton;
  } else if (absl::GetFlag(FLAGS_optimizer) == "lbfgs") {
    optimizer = new optim::Lbfgs;
  } else {
    AX_LOG(FATAL) << "Unknown optimizer: " << absl::GetFlag(FLAGS_optimizer);
  }

  /************************* SECT: Run Optimize *************************/
  auto start = utils::GetCurrentTimeNanos();
  optimizer->SetOptions(opt);
  // AX_LOG(INFO) << "Initial : " << x0.transpose();
  AX_LOG(INFO) << "Optimizer Options: " << std::endl << optimizer->GetOptions();
  auto solution = optimizer->Optimize(*prob, x0);

  auto end = utils::GetCurrentTimeNanos();

  AX_LOG(INFO) << "Time: " << (end - start) / 1e6 << " ms";
  AX_LOG(INFO) << "Optimization finished in " << solution.n_iter_ << " iterations";
  AX_LOG(INFO) << "Solution: " << solution.x_opt_.transpose();
  AX_LOG(INFO) << "Accurate: " << optimal.transpose();
  if (!solution.converged_) {
    AX_LOG(ERROR) << "Optimization failed to converge";
  } else {
    AX_LOG(INFO) << "Optimization converged";
  }
  delete optimizer;
  delete prob;
  clean_up();
  return 0;
}
