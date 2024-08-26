/**
 * @file main.cpp
 * @brief Benchmark for optimization algorithms. Only consider the gradient based methods.
 */

#include <benchmark/benchmark.h>

#include <ax/optim/common.hpp>
#include <iostream>

#include "ax/optim/linesearch/linesearch.hpp"
#include "ax/optim/optimizer_base.hpp"

namespace xx {
using namespace ax;
using namespace ax::math;
using namespace ax::optim;

template <typename Derived>
struct BenchmarkProblemBase {
  Real Energy(const Variable& x) const {
    return static_cast<const Derived*>(this)->Energy(x);
  }

  Gradient Grad(const Variable& x) const {
    return static_cast<const Derived*>(this)->Grad(x);
  }

  Variable InitialGuess() const {
    return static_cast<const Derived*>(this)->InitialGuess();
  }

  Variable OptimalSolution() const {
    return static_cast<const Derived*>(this)->OptimalSolution();
  }
};

struct Rosenbrock : public BenchmarkProblemBase<Rosenbrock> {
  static constexpr Real b = 10.0;

  Real Energy(const Variable& x) const {
    Real a = 1.0;
    Real x1 = x(0, 0);
    Real x2 = x(1, 0);
    return (a - x1) * (a - x1) + b * (x2 - x1 * x1) * (x2 - x1 * x1);
  }

  Gradient Grad(const Variable& x) const {
    Real a = 1.0;
    Real x1 = x(0, 0);
    Real x2 = x(1, 0);
    Gradient grad(2, 1);
    grad(0, 0) = 2 * (x1 - a) - 4 * b * x1 * (x2 - x1 * x1);
    grad(1, 0) = 2 * b * (x2 - x1 * x1);
    return grad;
  }

  Variable InitialGuess() const {
    Variable x(2, 1);
    x(0, 0) = x(1, 0) = 1.2;
    return x;
  }

  Variable OptimalSolution() const {
    Variable x(2, 1);
    x(0, 0) = 1.0;
    x(1, 0) = 1.0;
    return x;
  }
};
}  // namespace xx

std::vector<std::string> all_optimizers = {"kNonlinearCg", "kLbfgs"};
std::vector<std::string> all_linesearch = {"kBacktracking", "kWolfe"};

template <typename Problem, size_t kopt, size_t kls>
void BM_Optimize(benchmark::State& state) {
  using namespace ax;
  using namespace ax::optim;
  using namespace xx;

  Problem prob;

  OptProblem opt_prob;
  opt_prob.SetEnergy([&](const Variable& x) {
    return prob.Energy(x);
  });
  opt_prob.SetGrad([&](const Variable& x) {
    return prob.Grad(x);
  });
  auto optkind = utils::reflect_enum<OptimizerKind>(all_optimizers[kopt]).value();
  auto opt = OptimizerBase::Create(optkind);
  opt->SetOptions({{"linesearch", all_linesearch[kls]}});

  for (auto _ : state) {
    Variable x0 = prob.InitialGuess();
    OptResult result = opt->Optimize(opt_prob, x0);
    benchmark::DoNotOptimize(result);
    static std::once_flag of;
    std::call_once(of, [&]() {
      std::cout << "Optimal solution: " << prob.OptimalSolution().transpose() << std::endl;
      std::cout << "O: " << all_optimizers[kopt] << "; LS: " << all_linesearch[kls] << std::endl;
      std::cout << "Optimizer result: " << result.x_opt_.transpose() << std::endl;
      std::cout << "Optimizer iter: " << result.n_iter_ << std::endl;
    });
  }
}

BENCHMARK(BM_Optimize<xx::Rosenbrock, 1, 1>)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Optimize<xx::Rosenbrock, 1, 0>)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Optimize<xx::Rosenbrock, 0, 1>)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();
