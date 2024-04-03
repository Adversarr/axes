#include "ax/optim/linesearch/backtracking.hpp"

namespace ax::optim {

UPtr<LinesearchBase> LinesearchBase::Create(LineSearchKind kind) {
  switch (kind) {
    case LineSearchKind::kBacktracking:
      return std::make_unique<BacktrackingLinesearch>();
    default:
      return nullptr;
  }
}

}  // namespace ax::optim
