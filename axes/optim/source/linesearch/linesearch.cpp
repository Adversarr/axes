#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/wolfe.hpp"

namespace ax::optim {

UPtr<LinesearchBase> LinesearchBase::Create(LineSearchKind kind) {
  switch (kind) {
    case LineSearchKind::kBacktracking:
      return std::make_unique<Linesearch_Backtracking>();
    case LineSearchKind::kWolfe:
      return std::make_unique<Linesearch_Wofle>();
    default:
      return nullptr;
  }
}

utils::Opt LinesearchBase::GetOptions() const {
  utils::Opt opt = utils::Tunable::GetOptions();
  opt["max_iter"] = max_iter_;
  return opt;
}

void LinesearchBase::SetOptions(const utils::Opt& option) { AX_SYNC_OPT(option, idx, max_iter); }

}  // namespace ax::optim
