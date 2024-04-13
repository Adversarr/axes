#include "ax/graph/executor.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>
#include <stdexcept>

#include "ax/graph/node.hpp"
#include "ax/utils/status.hpp"

namespace ax::graph {

struct cycle_detector : public boost::dfs_visitor<> {
  template <class Edge, class Graph> void back_edge(Edge, Graph&) {
    throw std::runtime_error("Cycle detected");
  }
};

std::map<idx, std::set<idx>> GraphExecutorBase::DependencyMap() {
  std::map<idx, std::set<idx>> dependency_map;
  graph_.ForeachSocket([&dependency_map](Socket const* s) {
    idx from_node = s->input_->node_id_;
    idx to_node = s->output_->node_id_;
    dependency_map[to_node].insert(from_node);
  });
  return dependency_map;
}

bool GraphExecutorBase::HasCycle() {
  boost::adjacency_list<> g;

  graph_.ForeachSocket(
      [&g](Socket const* s) { boost::add_edge(s->output_->node_id_, s->input_->node_id_, g); });
  std::vector<idx> sorted;

  try {
    boost::topological_sort(g, std::back_inserter(sorted));
  } catch (boost::wrapexcept<boost::not_a_dag>&) {
    return true;
  }
  return false;
}

std::vector<idx> GraphExecutorBase::TopologicalSort() {
  std::set<idx> vertices;
  idx max_node_id = 0;
  graph_.ForeachNode([&](NodeBase const* n) {
    vertices.insert(n->GetId());
    max_node_id = n->GetId();
  });

  boost::adjacency_list<> g(max_node_id + 1);
  graph_.ForeachSocket(
      [&g](Socket const* s) { boost::add_edge(s->output_->node_id_, s->input_->node_id_, g); });

  std::vector<idx> sorted;

  try {
    boost::topological_sort(g, std::back_inserter(sorted));
  } catch (boost::wrapexcept<boost::not_a_dag>&) {
    return {};
  }
  std::vector<idx> result;

  for (auto id : sorted) {
    if (vertices.contains(id)) {
      result.push_back(id);
    }
  }
  return result;
}

Status GraphExecutorBase::Execute(idx beg, idx end) {
  if (beg > end) {
    return utils::InvalidArgumentError("Invalid frame range");
  }


  auto sorted = TopologicalSort();
  if (graph_.GetNumNodes() > 0 && sorted.empty()) {
    return utils::FailedPreconditionError("Exist a cycle in your input graph");
  }

  graph_.EnsurePayloads();
  auto status = utils::OkStatus();

  for (auto node_id: sorted) {
    AX_DLOG(INFO) << "Precompute " << node_id;
    auto node = graph_.GetNode(node_id);
    if (auto status = node->PreCompute(); !status.ok()) {
      AX_LOG(ERROR) << "Failed to precompute node " << node_id << ": " << status;
      return status;
    }
  }

  for (idx frame_id = beg; frame_id < end; ++frame_id) {
    graph_.ForeachNode([&](NodeBase* n) { status.Update(n->PreApply(frame_id)); });

    if (!status.ok()) {
      AX_LOG(ERROR) << "Failed to run pre-apply!" << status.message();
    }

    for (auto node_id : sorted) {
      auto node = graph_.GetNode(node_id);
      AX_DLOG(INFO) << "Executing node " << node_id;
      if (auto status = node->Apply(frame_id); !status.ok()) {
        AX_LOG(ERROR) << "Failed to execute node " << node_id << ": " << status;
        return status;
      }
    }

    graph_.ForeachNode([&](NodeBase* n) { status.Update(n->PostApply(frame_id)); });
    if (!status.ok()) {
      AX_LOG(ERROR) << "Failed to run post-apply!" << status.message();
    }
  }

  for (auto node_id: sorted) {
    auto node = graph_.GetNode(node_id);
    if (auto status = node->PostCompute(); !status.ok()) {
      AX_LOG(ERROR) << "Failed to postcompute node " << node_id << ": " << status;
      return status;
    }
  }

  AX_RETURN_OK();
}

}  // namespace ax::graph
