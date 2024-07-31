#pragma once
#include "graph.hpp"
#include <set>
#include <map>
#include "ax/utils/enum_refl.hpp"

namespace ax::graph {

BOOST_DEFINE_ENUM_CLASS(GraphExecuteStage, 
  kIdle,

  kPrePreApply,
  kPreApplyDone,
  kPreApplyError,

  kApplyRunning,
  kApplyDone,
  kApplyError,

  kPrePostApply,
  kPostApplyDone,
  kPostApplyError,

  kDone
);

class GraphExecutorBase {
public:
  virtual ~GraphExecutorBase() = default;
  GraphExecutorBase(Graph& graph) : graph_(graph) {}

  virtual void Execute();

  void Begin();
  void End();
  void WorkOnce();
  GraphExecuteStage GetStage() const { return stage_; }
  std::map<idx, std::set<idx>> DependencyMap();
  std::vector<idx> TopologicalSort();
  bool HasCycle();
  Graph& GetGraph() { return graph_; }

  void SetEnd(idx end) { end_ = end; }
  idx GetCurrentFrame() const { return current_frame_id_; }

  void CleanUpGraph();

  virtual void PreApply();

  virtual void PostApply();

  virtual void Apply(idx frame_id);

  virtual void LoopApply(idx end);

protected:
  Graph& graph_;
  std::vector<idx> toposort_;
  idx current_frame_id_ = 0;
  idx end_ = 1;
  GraphExecuteStage stage_ = GraphExecuteStage::kIdle;
};

}
