#include <absl/flags/flag.h>
#include <absl/flags/internal/flag.h>

#include <cstdlib>
#include <ios>
#include <memory>
#include <utility>

#include "ax/core/common.hpp"
#include "ax/core/echo.hpp"
#include "ax/core/init.hpp"
#include "ax/math/io.hpp"
#include "ax/optim/common.hpp"
#include "ax/optim/optimizer_base.hpp"
#include "ax/optim/optimizers/fista.hpp"
#include "ax/optim/optimizers/gd.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/iota.hpp"
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
ABSL_FLAG(real, mu, 1E-2, "l1 regularity");
ABSL_FLAG(real, tol_grad, 1E-6, "Tolerance for gradient change");
ABSL_FLAG(real, lr, 1E-3, "Learning rate");
ABSL_FLAG(std::string, linesearch, "backtracking", "Line search method: backtracking or wolfe");
ABSL_FLAG(std::string, export, "", "Export result to file");
ABSL_FLAG(bool, fista, false, "Enable FISTA");
ABSL_FLAG(bool, strong_wolfe, false, "Enable strong Wolfe condition");
ABSL_FLAG(bool, mono, false, "Fista step length is mono");

int main(int argc, char** argv) {
  ax::init(argc, argv);

  // Problem definition
  xx::SPLR splr = xx::load_from_file(utils::get_asset("/temporaries/a9a.txt"));
  splr.lambda_ = 0.5 / xx::PREDEFINED_M;
  splr.mu_ = absl::GetFlag(FLAGS_mu);

  std::cout << "lambda: " << splr.lambda_ << std::endl;
  std::cout << "mu: " << splr.mu_ << std::endl;
  OptProblem prob;

  vecxr x0 = vecxr::Zero(xx::PREDEFINED_FEAT);
  real lr = absl::GetFlag(FLAGS_lr);
  // Optimizer
  std::string opt_name = absl::GetFlag(FLAGS_optimizer);

  // History data recording:
  std::vector<math::vecxr> x_history;
  std::vector<real> tk_history;
  std::vector<real> energy_history;

  prob.SetProximator([&](vecxr const& x, real lambda) -> math::vecxr {
        tk_history.push_back(lambda);
        return xx::l1_proximator(x, splr.mu_ * lambda);
      })
      .SetGrad(
          [&](vecxr const& x) -> vecxr { return splr.Gradient_Loss(x) + splr.Gradient_Loss_L2(x); })
      .SetEnergy([&](vecxr const& x) -> real { return splr.Energy_Loss(x) + splr.Energy_L2(x); })
      .SetConvergeVar(nullptr);

  if (!absl::GetFlag(FLAGS_fista)) {
    optimizer = std::make_unique<GradientDescent>();
    GradientDescent* gd = dynamic_cast<GradientDescent*>(optimizer.get());
    gd->SetLearningRate(lr);
    if (absl::GetFlag(FLAGS_linesearch) == "backtracking") {
      std::cout << "Using backtracking line search.\n";
      auto p = LinesearchBase::Create(ax::optim::LineSearchKind::kBacktracking);
      p->SetOptions(
          {{"initial_step_length", 1}, {"step_shrink_rate", 0.6}, {"required_descent_rate", 0.3}});
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
  } else {
    optimizer = std::make_unique<Fista>();
    Fista* fista = dynamic_cast<Fista*>(optimizer.get());
    fista->SetMonotonic(absl::GetFlag(FLAGS_mono));
  }

  optimizer->SetOptions({{"max_iter", absl::GetFlag(FLAGS_max_iter)},
                         {"tol_var", absl::GetFlag(FLAGS_tol_var)},
                         {"tol_grad", absl::GetFlag(FLAGS_tol_grad)},
                         {"verbose", absl::GetFlag(FLAGS_verbose)}});
  AX_LOG(WARNING) << "Optimizer Options: " << optimizer->GetOptions();

  prob.SetVerbose([&, verb = absl::GetFlag(FLAGS_verbose)](idx iter, vecxr const& x, real energy) {
    real e = splr.Energy(x);
    if (iter % 100 == 0 || verb) {
      std::cout << "Iter " << iter << ": " << e << std::endl;
      // print sparsity.
      idx n = 0;
      for (idx i = 0; i < x.size(); ++i) {
        if (std::abs(x(i)) > 1E-7) {
          ++n;
        }
      }
      std::cout << "Sparsity: " << n << "/" << x.size() << std::endl;
    }
    x_history.push_back(x);
    energy_history.push_back(e);
  });

  // Initial guess
  AX_LOG(INFO) << "Problem loaded.\n";
  OptResult result = optimizer->Optimize(prob, x0);

  // Reason for termination
  AX_LOG(WARNING) << "Converged Grad: " << std::boolalpha << result.converged_grad_;
  AX_LOG(WARNING) << "Converged Var: " << std::boolalpha << result.converged_var_;
  AX_LOG(WARNING) << "Iter: " << result.n_iter_;
  AX_LOG(WARNING) << "Energy: " << splr.Energy(result.x_opt_);

  if (absl::GetFlag(FLAGS_fista)) {
    Fista* fista = dynamic_cast<Fista*>(optimizer.get());
    tk_history = fista->tk_;
  }

  // write output.
  std::string const export_prefix = absl::GetFlag(FLAGS_export);
  // solution.
  {
    math::matxxr x(xx::PREDEFINED_FEAT, x_history.size());
    for (auto i : utils::iota(x_history.size())) {
      x.col(i) = x_history[i];
    }
    math::write_npy_v10(export_prefix + "_x.npy", x);
  }
  {
    // export |xk-xk+1|/tk
    math::vecxr diffs(tk_history.size());
    size_t n = std::min(x_history.size() - 1, tk_history.size());
    for (auto i : utils::iota(n)) {
      diffs(i) = (x_history[i] - x_history[i + 1]).norm() / tk_history[i];
    }
    math::write_npy_v10(export_prefix + "_diff.npy", diffs);
  }
  // export energy.
  {
    math::vecxr energies(energy_history.size());
    for (auto i : utils::iota(energy_history.size())) {
      energies(i) = energy_history[i];
    }
    math::write_npy_v10(export_prefix + "_energy.npy", energies);
  }


  ax::clean_up();
  return EXIT_SUCCESS;
}
