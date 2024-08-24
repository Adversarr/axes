#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/math/utils/formatting.hpp"
#include "ax/optim/optimizers/gd.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/optim/test_problems.hpp"
#include "ax/utils/time.hpp"

using namespace ax;
Index n = 2;
math::RealMatrixX A;
math::RealVectorX b;

int main(int argc, char** argv) {
  get_program_options().add_options()("problem", "Problem to solve",
                                      cxxopts::value<std::string>()->default_value("sp_lstsq"))(
      "optimizer", "Optimizer", cxxopts::value<std::string>()->default_value("lbfgs"))(
      "verbose", "Verbose output", cxxopts::value<bool>()->default_value("false"))(
      "dof", "Degrees of freedom", cxxopts::value<Index>()->default_value("2"))(
      "pcg_strategy", "PCG strategy",
      cxxopts::value<std::string>()->default_value("kFletcherReeves"));

  init(argc, argv);

  std::shared_ptr<optim::OptimizerBase> optimizer;
  std::shared_ptr<optim::OptProblem> prob;
  optim::Variable optimal, x0;

  n = get_parse_result()["dof"].as<Index>();
  bool verbose = get_parse_result()["verbose"].as<bool>();
  std::string problem = get_parse_result()["problem"].as<std::string>();
  std::string optimizer_name = get_parse_result()["optimizer"].as<std::string>();

  /************************* SECT: Optimizer Options *************************/
  utils::Options opt{
    {"verbose", verbose},
    {"max_iter", static_cast<Index>(200)},
    {"linesearch", "kBacktracking"},
    {"linesearch_opt", utils::Options{{"required_descent_rate", 1e-4}, {"verbose", true}}}};

  /************************* SECT: Create Optimizer *************************/
  if (optimizer_name == "newton") {
    // optimizer = new optim::Optimizer_Newton;
    optimizer = std::make_shared<optim::Optimizer_Newton>();
  } else if (optimizer_name == "lbfgs") {
    optimizer = std::make_shared<optim::Optimizer_Lbfgs>();
  } else if (optimizer_name == "gd") {
    optimizer = std::make_shared<optim::Optimizer_GradientDescent>();
  } else if (optimizer_name == "pcg") {
    optimizer = optim::OptimizerBase::Create(optim::OptimizerKind::kNonlinearCg);
    optimizer->SetOptions({
      {"strategy", get_parse_result()["pcg_strategy"].as<std::string>()},
    });
  } else {
    AX_CRITICAL("Unknown optimizer: {}", optimizer_name);
    abort();
  }
  optimizer->SetOptions(opt);
  AX_INFO("Optimizer Options: \n{}", fmt::streamed(optimizer->GetOptions()));

  /************************* SECT: Setup Problems *************************/
  if (problem == "rosenbrock") {
    prob = std::make_shared<optim::test::RosenbrockProblem>();
    x0 = math::RealVectorX::Constant(n, 1.2);
    optimal = optim::test::RosenbrockProblem{}.Optimal(x0);
  } else if (problem == "lstsq") {
    A = math::RealMatrixX::Random(n, n);
    A = (math::eye(n) + A * A.transpose()).eval();
    b = math::RealVectorX::Random(n);
    prob = std::make_shared<optim::test::LeastSquareProblem>(A, b);
    optimal = optim::test::LeastSquareProblem{A, b}.Optimal(b);
    x0.setRandom(n, 1);
  } else if (problem == "sp_lstsq") {
    math::SparseCOO A_sparse;
    A_sparse.reserve(n * 10);
    for (Index i = 0; i < n; ++i) {
      for (Index j = 0; j < 9; ++j) {
        A_sparse.push_back({i, rand() % n, Real(rand() % 100) / 100.0});
      }
      A_sparse.push_back({i, i, Real(1.0)});
    }
    math::RealSparseMatrix A2{n, n};
    b.setRandom(n);
    A2.setFromTriplets(A_sparse.begin(), A_sparse.end());
    A2.makeCompressed();
    A2 = A2.transpose() * A2;
    for (Index i = 0; i < n; ++i) {
      A2.coeffRef(i, i) += 1;
    }
    A2 /= 10;
    auto* sp_lstsq = new optim::test::SparseLeastSquareProblem(A2, b);
    x0 = math::ones(n, 1) * 1.2;
    optimal = sp_lstsq->Optimal(x0);
    prob = std::shared_ptr<optim::test::SparseLeastSquareProblem>(sp_lstsq);
  } else {
    AX_CRITICAL("Unknown problem: {}", problem);
    abort();
  }
  prob->SetVerbose([verbose](Index iter, const optim::Variable& x, Real f) {
    if (verbose) {
      AX_INFO("Iter: {}, x: {}, f: {}", iter, x, f);
    }
  });

  /************************* SECT: Run Optimize *************************/
  auto start = utils::get_current_time_nanos();
  AX_INFO("Initial : {}", x0);
  auto solution = optimizer->Optimize(*prob, x0);

  auto end = utils::get_current_time_nanos();
  AX_INFO("Time: {} ms", (end - start) / 1e6);
  AX_INFO("Optimization finished in {} iterations", solution.n_iter_);
  AX_INFO("Solution: {}", solution.x_opt_);
  AX_INFO("Accurate: {}", optimal);
  if (!solution.converged_) {
    AX_ERROR("Optimization failed to converge");
  } else {
    AX_INFO("Optimization converged");
  }

  clean_up();
  return 0;
}
