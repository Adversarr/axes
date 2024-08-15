#include "ax/graph/executor.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>
#include <stdexcept>

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"

namespace ax::graph {

bool GraphExecutorBase::HasCycle() const { return graph_.HasCycle(); }

std::vector<size_t> GraphExecutorBase::TopologicalSort() const { return graph_.TopologyOrder(); }

void GraphExecutorBase::Execute() {
  if (end_ < 0) {
    throw make_invalid_argument("Invalid frame range");
  }
  this->LoopApply(end_);
}

void GraphExecutorBase::CleanUpGraph() {
  // graph_.ForeachNode([](NodeBase* n) { n->CleanUp(); });
}

void GraphExecutorBase::LoopApply(idx end) {
  for (idx frame_id = current_frame_id_; frame_id <= end; ++frame_id) {
    Apply(frame_id);
  }
}

void GraphExecutorBase::Apply(idx frame_id) {
  // for (auto node_id : toposort_) {
  //   // TODO: try except
  //   node->Apply(frame_id);
  // }
}

void GraphExecutorBase::ExecuteOnce() {}

}  // namespace ax::graph
