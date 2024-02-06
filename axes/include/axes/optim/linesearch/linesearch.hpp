#pragma once

#include "axes/optim/common.hpp"
#include "axes/utils/opt.hpp"

namespace ax::optim {

enum class LineSearchKind { kBacktracking };

class LineSearchBase {
public:
  virtual ~LineSearchBase() = default;

  virtual OptResult Optimize(OptProblem const& prob, math::vecxr const& x0, math::vecxr const& dir, utils::Opt const& options) = 0;

  static utils::uptr<LineSearchBase> Create(LineSearchKind kind);

protected:
  idx max_iter_{20};
};

}  // namespace ax::optim
#include "axes/utils/enum_refl.hpp"

AX_ENUM_REFL_BEGIN(ax::optim::LineSearchKind)
AX_ENUM_STATE(kBacktracking, Backtracking)
AX_ENUM_REFL_END();
