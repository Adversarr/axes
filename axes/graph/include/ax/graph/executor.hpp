#pragma once
#include <map>
#include <set>

#include "ax/utils/enum_refl.hpp"
#include "common.hpp"

namespace ax::graph {

/**
 * The stage of the graph execution.
 * 1. Idle: indicates that the graph executor is not running, and can launch a new execution.
 * 2. Running: indicates that the graph executor is running, and the nodes are being executed.
 *    but the target frame is not reached.
 * 3. Done: indicates that the graph executor has finished the execution of the target frame.
 *    and the target frame is reached, successfully.
 * 4. Error: indicates that the graph executor has finished the execution of the target frame.
 *    you should clear the error bit and then enter Idle state.
 */
AX_DEFINE_ENUM_CLASS(GraphExecuteStage, kIdle, kRunning, kDone, kError);

class GraphExecutorBase {
public:
  virtual ~GraphExecutorBase() = default;
  explicit GraphExecutorBase(Graph& graph) : graph_(graph) {}

  void Execute();
  void ExecuteOnly(Index frame_id);
  virtual void ExecuteOnce(Index frame_id);

  GraphExecuteStage GetStage() const noexcept { return stage_; }

  std::vector<size_t> TopologicalSort() const;

  bool HasCycle() const;

  Graph& GetGraph() const noexcept;

  void SetEnd(Index end) { end_ = end; }

  Index GetCurrentFrame() const { return current_frame_id_; }

protected:
  Graph& graph_;

  Context run_ctx_;
  std::vector<Index> toposort_;
  Index current_frame_id_ = 0;
  Index end_ = 1;
  GraphExecuteStage stage_ = GraphExecuteStage::kIdle;
  std::string error_msg_;
};

}  // namespace ax::graph
