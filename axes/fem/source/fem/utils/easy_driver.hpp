#pragma once
#include "ax/fem/problem.hpp"
#include "ax/fem/state.hpp"
#include "ax/fem/utils/prune_dbc.hpp"

namespace ax::fem {

class EasyDriver {
public:
  explicit EasyDriver(std::shared_ptr<Mesh> mesh);

  void Solve();

  Problem& GetProblem() { return prob_; }

  std::shared_ptr<State> GetState() { return prob_.GetState(); }

private:
  PruneDirichletBc pruner_;
  Problem prob_;
};

}  // namespace ax::fem