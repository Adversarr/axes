#include "ax/optim/optimizer_base.hpp"

#include "ax/core/logging.hpp"
#include "ax/optim/optimizers/gd.hpp"
#include "ax/optim/optimizers/lbfgs.hpp"
#include "ax/optim/optimizers/newton.hpp"
#include "ax/utils/opt.hpp"

namespace ax::optim {

std::unique_ptr<OptimizerBase> OptimizerBase::Create(OptimizerKind k) {
  switch (k) {
    case OptimizerKind::kNewton:
      return std::make_unique<Optimizer_Newton>();
    case OptimizerKind::kLbfgs:
      return std::make_unique<Optimizer_Lbfgs>();
    case OptimizerKind::kGradientDescent:
      return std::make_unique<Optimizer_GradientDescent>();
    default:
      return nullptr;
  }
}

void OptimizerBase::SetMaxIter(idx max_iter) {
  AX_THROW_IF_LT(max_iter, 1, "max_iter must be positive");
  max_iter_ = max_iter;
}

void OptimizerBase::SetTolVar(real tol_var) {
  AX_THROW_IF_LT(tol_var, 0, "tol_var must be non-negative");
  tol_var_ = tol_var;
}

void OptimizerBase::SetTolGrad(real tol_grad) {
  AX_THROW_IF_LT(tol_grad, 0, "tol_grad must be non-negative");
  tol_grad_ = tol_grad;
}

void OptimizerBase::SetOptions(utils::Options const& options) {
  AX_SYNC_OPT_IF(options, idx, max_iter) {
    AX_THROW_IF_LT(max_iter_, 1, "max_iter must be positive");
  }

  AX_SYNC_OPT_IF(options, real, tol_var) {
    AX_THROW_IF_LT(tol_var_, 0, "tol_var must be non-negative");
  }

  AX_SYNC_OPT_IF(options, real, tol_grad) {
    AX_THROW_IF_LT(tol_grad_, 0, "tol_grad must be non-negative");
  }

  AX_SYNC_OPT(options, bool, verbose);
  // AX_SYNC_OPT(options, bool, record_trajectory);
  utils::Tunable::SetOptions(options);
}

// void OptimizerBase::RecordTrajectory(math::vecxr const& x, math::vecxr const& grad,
//                                      real energy) const {
//   if (record_trajectory_) {
//     x_history_.push_back(x);
//     grad_history_.push_back(grad);
//     energy_history_.push_back(energy);
//   }
// }
//
// void OptimizerBase::ClearTrajectory() const {
//   x_history_.clear();
//   grad_history_.clear();
//   energy_history_.clear();
// }

utils::Options OptimizerBase::GetOptions() const {
  utils::Options options{
      {"max_iter", max_iter_},
      {"tol_var", tol_var_},
      {"tol_grad", tol_grad_},
      {"verbose", idx(verbose_ ? 1 : 0)},
      // {"record_trajectory", idx(record_trajectory_ ? 1 : 0)},
  };
  return options;
}

}  // namespace ax::optim
