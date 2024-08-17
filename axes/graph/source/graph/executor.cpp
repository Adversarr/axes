#include "ax/graph/executor.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>
#include <range/v3/view/enumerate.hpp>
#include <stdexcept>

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"

namespace ax::graph {

bool GraphExecutorBase::HasCycle() const { return graph_.HasCycle(); }

Graph& GraphExecutorBase::GetGraph() const noexcept { return graph_; }

std::vector<size_t> GraphExecutorBase::TopologicalSort() const { return graph_.TopologyOrder(); }

void GraphExecutorBase::Execute() {
  if (stage_ != GraphExecuteStage::kIdle) {
    AX_ERROR("GraphExecutor is not in idle state, current stage: {}",
             utils::reflect_name(stage_).value_or("???"));
  }
  stage_ = GraphExecuteStage::kRunning;
  for (Index i = 0; i < end_; ++i) {
    AX_INFO("Executing frame: {}", i);
    ExecuteOnce(i);
    if (stage_ == GraphExecuteStage::kError) {
      AX_ERROR("Failed to execute frame: {}", i);
      break;
    }
  }
  if (stage_ == GraphExecuteStage::kRunning) {
    stage_ = GraphExecuteStage::kDone;
  }
}

void GraphExecutorBase::ExecuteOnly(Index frame_id) {
  if (stage_ != GraphExecuteStage::kIdle) {
    AX_ERROR("GraphExecutor is not in idle state, current stage: {}",
             utils::reflect_name(stage_).value_or("???"));
  }
  stage_ = GraphExecuteStage::kRunning;
  AX_INFO("Executing frame: {}", frame_id);
  ExecuteOnce(frame_id);
  if (stage_ == GraphExecuteStage::kError) {
    AX_ERROR("Failed to execute frame: {}", frame_id);
  }

  if (stage_ == GraphExecuteStage::kRunning) {
    stage_ = GraphExecuteStage::kDone;
  }
}

void GraphExecutorBase::ExecuteOnce(Index frame_id) {
  stage_ = GraphExecuteStage::kRunning;
  error_msg_.clear();

  graph_.TopologySort();
  run_ctx_.Clear();
  run_ctx_.PushStack();
  run_ctx_.Emplace<Index>("frame_index", frame_id);

  for (auto [i, n] : ranges::views::enumerate(graph_.GetNodes())) {
    if (n) {
      try {
        n->operator()(run_ctx_);
      } catch (std::exception const& e) {
        AX_ERROR("Failed to execute node {} with error: {}", n->GetDescriptor().Name(), e.what());
        error_msg_ = e.what();
        stage_ = GraphExecuteStage::kError;
        return;
      }
    }
  }

  size_t stack_cnt = run_ctx_.StackSize();
  if (stack_cnt > 1) {
    AX_ERROR("Stack count is not balanced, stack count: {}", stack_cnt);
    stage_ = GraphExecuteStage::kError;
    error_msg_ = "Stack count is not balanced";
  }
}

}  // namespace ax::graph
