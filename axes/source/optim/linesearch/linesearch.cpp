#include "axes/optim/linesearch/backtracking.hpp"

namespace ax::optim {

utils::uptr<LinesearchBase> LinesearchBase::Create(LineSearchKind kind) {
  switch (kind) {
    case LineSearchKind::kBacktracking:
      return std::make_unique<BacktrackingLinesearch>();
    default:
      return nullptr;
  }
}

}  // namespace ax::optim
