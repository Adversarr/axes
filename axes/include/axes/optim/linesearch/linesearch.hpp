#pragma once

#include "axes/optim/common.hpp"

namespace ax::optim {

class LineSearchBase {
public:
  LineSearchBase(OptProblem& problem) : problem_{problem} {}

  virtual ~LineSearchBase() = default;

  virtual OptResult Optimize(math::vecxr const& x0, math::vecxr const& dir) = 0;

protected:
  idx max_iter_{20};

  OptProblem& problem_;
};

}  // namespace ax::optim
