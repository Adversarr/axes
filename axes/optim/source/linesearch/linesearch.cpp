#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/exact.hpp"
#include "ax/optim/linesearch/null_ls.hpp"
#include "ax/optim/linesearch/wolfe.hpp"

namespace ax::optim {

std::unique_ptr<LinesearchBase> LinesearchBase::Create(LineSearchKind kind) {
  switch (kind) {
    case LineSearchKind::Backtracking:
      return std::make_unique<Linesearch_Backtracking>();
    case LineSearchKind::Wolfe:
      return std::make_unique<Linesearch_Wofle>();
    case LineSearchKind::Exact:
      return std::make_unique<Linesearch_Exact>();
    case LineSearchKind::Null:
      return std::make_unique<Linesearch_Null>();
    default:
      return nullptr;
  }
}

utils::Options LinesearchBase::GetOptions() const {
  utils::Options opt = utils::Tunable::GetOptions();
  opt["max_iter"] = max_iter_;
  opt["min_step_size"] = min_step_size_;
  opt["max_step_size"] = max_step_size_;
  opt["initial_step_size"] = initial_step_size_;
  opt["verbose"] = verbose_;
  return opt;
}

void LinesearchBase::SetOptions(const utils::Options& option) {
  AX_SYNC_OPT(option, Index, max_iter);
  AX_SYNC_OPT(option, Real, min_step_size);
  AX_SYNC_OPT(option, Real, max_step_size);
  AX_SYNC_OPT(option, Real, initial_step_size);
  AX_SYNC_OPT(option, bool, verbose);
}

}  // namespace ax::optim
