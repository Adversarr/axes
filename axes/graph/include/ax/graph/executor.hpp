#pragma once
#include <map>
#include <set>

#include "ax/utils/enum_refl.hpp"
#include "common.hpp"

namespace ax::graph {

AX_DEFINE_ENUM_CLASS(GraphExecuteStage, kIdle, kApplyRunning, kApplyDone, kApplyError, kDone);

class GraphExecutorBase {
public:
  virtual ~GraphExecutorBase() = default;
  explicit GraphExecutorBase(Graph& graph) : graph_(graph) {}

  virtual void Execute();
  void ExecuteOnce();

  GraphExecuteStage GetStage() const noexcept { return stage_; }

  std::vector<size_t> TopologicalSort() const;
  bool HasCycle() const;
  Graph& GetGraph() { return graph_; }

  void SetEnd(idx end) { end_ = end; }
  idx GetCurrentFrame() const { return current_frame_id_; }

  void CleanUpGraph();

  virtual void Apply(idx frame_id);
  virtual void LoopApply(idx end);

protected:
  Graph& graph_;

  Context run_ctx_;
  std::vector<idx> toposort_;
  idx current_frame_id_ = 0;
  idx end_ = 1;
  GraphExecuteStage stage_ = GraphExecuteStage::kIdle;
};

}  // namespace ax::graph
