#include "ax/optim/linesearch/backtracking.hpp"
#include "ax/optim/linesearch/wolfe.hpp"

namespace ax::optim {

std::unique_ptr<LinesearchBase> LinesearchBase::Create(LineSearchKind kind) {
  switch (kind) {
    case LineSearchKind::kBacktracking:
      return std::make_unique<Linesearch_Backtracking>();
    case LineSearchKind::kWolfe:
      return std::make_unique<Linesearch_Wofle>();
    default:
      return nullptr;
  }
}

utils::Options LinesearchBase::GetOptions() const {
  utils::Options opt = utils::Tunable::GetOptions();
  opt["max_iter"] = max_iter_;
  return opt;
}

void LinesearchBase::SetOptions(const utils::Options& option) { AX_SYNC_OPT(option, idx, max_iter); }

}  // namespace ax::optim
