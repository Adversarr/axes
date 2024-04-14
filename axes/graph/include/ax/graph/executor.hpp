#pragma once
#include "graph.hpp"
#include <set>
#include <map>

namespace ax::graph {

class GraphExecutorBase {
public:
  virtual ~GraphExecutorBase() = default;
  GraphExecutorBase(Graph& graph) : graph_(graph) {}

  virtual Status Execute(idx end);

  std::map<idx, std::set<idx>> DependencyMap();
  std::vector<idx> TopologicalSort();
  bool HasCycle();

protected:
  Graph& graph_;
};

}
