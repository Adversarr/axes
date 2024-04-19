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

Status GraphExecutorBase::Execute() {
  if (end_ < 0) {
    return utils::InvalidArgumentError("Invalid frame range");
  }
  Status s = utils::OkStatus();
  s.Update(this->PreApply());
  if (!s.ok()) {
    return s;
  }
  s.Update(this->LoopApply(end_));
  if (!s.ok()) {
    return s;
  }

  s.Update(this->PostApply());
  if (!s.ok()) {
    return s;
  }
  AX_RETURN_OK();
}

void GraphExecutorBase::CleanUpGraph() {
  graph_.ForeachNode([](NodeBase* n) { n->CleanUp(); });
}

Status GraphExecutorBase::PreApply() {
  toposort_ = TopologicalSort();
  if (graph_.GetNumNodes() > 0 && toposort_.empty()) {
    return utils::FailedPreconditionError("Exist a cycle in your input graph");
  }

  for (auto node_id: toposort_) {
    AX_DLOG(INFO) << "Precompute " << node_id;
    auto node = graph_.GetNode(node_id);
    if (auto status = node->PreApply(); !status.ok()) {
      AX_LOG(ERROR) << "Failed to precompute node " << node_id << ": " << status;
      return status;
    }
  }

  AX_RETURN_OK();
}

Status GraphExecutorBase::PostApply() {
  for (auto node_id: toposort_) {
    auto node = graph_.GetNode(node_id);
    if (auto status = node->PostApply(); !status.ok()) {
      AX_LOG(ERROR) << "Failed to postcompute node " << node_id << ": " << status;
      return status;
    }
  }
  AX_RETURN_OK();
}

Status GraphExecutorBase::LoopApply(idx end) {
  for (idx frame_id = current_frame_id_; frame_id <= end; ++frame_id) {
    if (auto status = Apply(frame_id); !status.ok()) {
      AX_LOG(ERROR) << "Failed to execute frame " << frame_id << ": " << status;
      return status;
    }
  }
  AX_RETURN_OK();
}

Status GraphExecutorBase::Apply(idx frame_id) {
  for (auto node_id : toposort_) {
    auto node = graph_.GetNode(node_id);
    if (auto status = node->Apply(frame_id); !status.ok()) {
      AX_LOG(ERROR) << "Failed to execute node " << node_id << ": " << status;
      return status;
    }
  }
  AX_RETURN_OK();
}

Status GraphExecutorBase::Begin() {
  if (stage_ != GraphExecuteStage::kIdle) {
    auto optname = utils::reflect_name(stage_);
    return utils::CancelledError("GraphExecutor is not in idle state" + optname.value_or(""));
  }
  stage_ = GraphExecuteStage::kPrePreApply;
  current_frame_id_ = 0;
  AX_RETURN_OK();
}

void GraphExecutorBase::End() {
  if (stage_ != GraphExecuteStage::kDone) {
    auto optname = utils::reflect_name(stage_);
    AX_LOG(WARNING) << "GraphExecutor is not in post apply done state" << optname.value_or("");
  }
  stage_ = GraphExecuteStage::kIdle;
}

Status GraphExecutorBase::WorkOnce() {
  switch (stage_) {
    case GraphExecuteStage::kIdle:
      // Nothing to do.
      AX_RETURN_OK();
    /* PRE APPLY */
    case GraphExecuteStage::kPrePreApply: {
      Status s = PreApply();
      if (!s.ok()) {
        stage_ = GraphExecuteStage::kPreApplyError;
        AX_LOG(ERROR) << "Failed to pre apply: " << s;
        return s;
      } else {
        stage_ = GraphExecuteStage::kPreApplyDone;
      }
      AX_RETURN_OK();
    }
    case GraphExecuteStage::kPreApplyDone:
      current_frame_id_ = 0;
    /* APPLY */
    case GraphExecuteStage::kApplyRunning: {
      // Although current_frame_id_ may be larger than end_, for example, 0 = current_frame_id_ = end_
      // We still apply the first time.
      Status s = Apply(current_frame_id_);
      if (!s.ok()) {
        stage_ = GraphExecuteStage::kApplyError;
        AX_LOG(ERROR) << "Frame id: " << current_frame_id_ << "Failed to apply: " << s;
        return s;
      } else {
        ++current_frame_id_;
        if (current_frame_id_ >= end_) /* Apply all done */ {
          stage_ = GraphExecuteStage::kApplyDone;
        } else /* Go to next frame */ {
          stage_ = GraphExecuteStage::kApplyRunning;
        }
      }
      AX_RETURN_OK();
    }
    case GraphExecuteStage::kApplyDone:
      stage_ = GraphExecuteStage::kPostApplyDone;
      return PostApply();
    /* POST-APPLY */
    case GraphExecuteStage::kPrePostApply: {
      Status s = PostApply();
      if (!s.ok()) {
        stage_ = GraphExecuteStage::kPostApplyError;
        AX_LOG(ERROR) << "Failed to post apply: " << s;
        return s;
      } else {
        stage_ = GraphExecuteStage::kPostApplyDone;
      }
      AX_RETURN_OK();
    }
    case GraphExecuteStage::kPostApplyDone:
      stage_ = GraphExecuteStage::kDone;
      return utils::OkStatus();

    // All the errors:
    case GraphExecuteStage::kPreApplyError:
      return utils::FailedPreconditionError("GraphExecutor is in pre-apply error state");
    case GraphExecuteStage::kApplyError:
      return utils::FailedPreconditionError("GraphExecutor is in apply error state");
    case GraphExecuteStage::kPostApplyError:
      return utils::FailedPreconditionError("GraphExecutor is in post-apply error state");

    default:
      return utils::FailedPreconditionError("GraphExecutor is in unknown state");
  }
}


}  // namespace ax::graph
