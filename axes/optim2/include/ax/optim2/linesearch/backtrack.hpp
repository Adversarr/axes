#pragma once
#include "base.hpp"

namespace ax::optim2 {

class LineSearch_Backtracking : public LineSearchBase {
public:
  LineSearch_Backtracking();

  LineSearchResult Optimize(LineSearchParam param) override;

  LineSearchKind GetKind() const override;
};

}  // namespace ax::optim2