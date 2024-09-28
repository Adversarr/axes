#pragma once

#include "base.hpp"

namespace ax::optim2 {

class LineSearch_Wolfe : public LineSearchBase {
public:
  LineSearchResult Optimize(LineSearchParam param) override;

  LineSearchKind GetKind() const override;
};

}  // namespace ax::optim2