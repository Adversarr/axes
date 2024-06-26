#include <absl/flags/flag.h>
#include <absl/flags/internal/flag.h>

#include <cstdlib>
#include <ios>
#include <memory>
#include <utility>

#include "ax/core/common.hpp"
#include "ax/core/echo.hpp"
#include "ax/core/init.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizer_base.hpp"
#include "ax/optim/optimizers/gd.hpp"
#include "ax/utils/asset.hpp"
#include "prob.hpp"
#include "prox_grad.hpp"

using namespace ax;
using namespace math;
using namespace optim;
UPtr<OptimizerBase> optimizer;

ABSL_FLAG(std::string, optimizer, "prox_grad", "Optimizer to use");
ABSL_FLAG(bool, verbose, false, "Verbose mode");
ABSL_FLAG(idx, max_iter, 1000, "Maximum number of iterations");
ABSL_FLAG(real, tol_var, 1E-6, "Tolerance for variable change");
ABSL_FLAG(real, tol_grad, 1E-6, "Tolerance for gradient change");
ABSL_FLAG(real, lr, 1E-3, "Learning rate");
ABSL_FLAG(std::string, linesearch, "backtracking", "Line search method: backtracking or wolfe");
ABSL_FLAG(std::string, export, "", "Export result to file");
ABSL_FLAG(bool, fista, false, "Enable FISTA");
ABSL_FLAG(bool, strong_wolfe, false, "Enable strong Wolfe condition");

int main(int argc, char** argv) {
  ax::init(argc, argv);

  // Problem definition
  xx::SPLR splr = xx::load_from_file(utils::get_asset("/temporaries/a9a.txt"));
  splr.lambda_ = 0.5 / xx::PREDEFINED_M;
  splr.mu_ = 1E-2;
  OptProblem prob;

  vecxr x0 = vecxr::Zero(splr.bA_.rows());
  real lr = absl::GetFlag(FLAGS_lr);
  // Optimizer
  std::string opt_name = absl::GetFlag(FLAGS_optimizer);
  if (opt_name == "prox_grad") {
    optimizer = std::make_unique<GradientDescent>();
    GradientDescent* gd = dynamic_cast<GradientDescent*>(optimizer.get());
    gd->SetProximator([&](vecxr const& x, real lambda) -> math::vecxr {
      return xx::l1_proximator(x, splr.mu_ * lambda);
    });
    prob.SetGrad(
        [&](vecxr const& x) -> vecxr { return splr.Gradient_Loss(x) + splr.Gradient_Loss_L2(x); });

    prob.SetEnergy([&](vecxr const& x) -> real { return splr.Energy_Loss(x) + splr.Energy_L2(x); });
    gd->SetLearningRate(lr);
    if (absl::GetFlag(FLAGS_linesearch) == "backtracking") {
      std::cout << "Using backtracking line search.\n";
      auto p = LinesearchBase::Create(ax::optim::LineSearchKind::kBacktracking);
      p->SetOptions(
          {{"initial_step_length", 1}, {"step_shrink_rate", 0.3}, {"required_descent_rate", 0.1}});
      gd->SetLineSearch(std::move(p));
    } else if (absl::GetFlag(FLAGS_linesearch) == "wolfe") {
      std::cout << "Using Wolfe line search.\n";
      auto p = LinesearchBase::Create(ax::optim::LineSearchKind::kWolfe);
      p->SetOptions({{"initial_step_length", 1},
                     {"step_shrink_rate", 0.3},
                     {"required_descent_rate", 0.1},
                     {"required_curvature_rate", 0.1}});
      gd->SetLineSearch(std::move(p));
    }

    // FISTA mode:
    gd->EnableFista(absl::GetFlag(FLAGS_fista));
  } else {
    throw std::runtime_error("Unknown optimizer: " + opt_name);
  }

  optimizer->SetOptions({{"max_iter", absl::GetFlag(FLAGS_max_iter)},
                         {"tol_var", absl::GetFlag(FLAGS_tol_var)},
                         {"tol_grad", absl::GetFlag(FLAGS_tol_grad)},
                         {"verbose", absl::GetFlag(FLAGS_verbose)}});
  AX_LOG(WARNING) << "Optimizer Options: " << optimizer->GetOptions();

  if (absl::GetFlag(FLAGS_verbose)) {
    prob.SetVerbose([&splr](idx iter, vecxr const& x, real energy) {
      if (iter % 100 == 0) {
        std::cout << "Iter " << iter << ": " << splr.Energy(x) << std::endl;
      }
    });
  }

  // Initial guess
  AX_LOG(INFO) << "Problem loaded.\n";
  OptResult result = optimizer->Optimize(prob, x0);

  // Reason for termination
  AX_LOG(WARNING) << "Converged Grad: " << std::boolalpha << result.converged_grad_;
  AX_LOG(WARNING) << "Converged Energy: " << std::boolalpha << result.converged_var_;
  AX_LOG(WARNING) << "Converged Iter: " << result.n_iter_;
  if (auto export_file = absl::GetFlag(FLAGS_export); !export_file.empty()) {
    std::ofstream file(export_file);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open file " + export_file);
    }
    // LINE 1: F optimal.
    auto const& x_opt = result.x_opt_;
    file << splr.Energy(x_opt) << std::endl;
    // LINE 2: x_opt.
    for (idx i = 0; i < x_opt.size(); ++i) {
      file << x_opt(i) << " ";
    }
    file << std::endl;
    // Line 3: gradient.
    auto grad = prob.EvalGrad(x_opt);
    for (idx i = 0; i < grad.size(); ++i) {
      file << grad(i) << " ";
    }
    file << std::endl;
    AX_LOG(INFO) << "Result exported to " << export_file;
  }
  ax::clean_up();
  return EXIT_SUCCESS;
}