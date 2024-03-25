#pragma once
#include "common.hpp"

namespace ax::graph {

using Id = size_t;

class Graph {
public:
  struct Impl;

  Graph() = default;

  /************************* SECT: Node Manipulations *************************/
  /**
   * @brief Add a node to the graph, with no connection.
   * @return The unique index of the node in the graph.
   */
  Id AddNode(std::unique_ptr<NodeBase> node);

  /**
   * @brief Remove a node from the graph.
   */
  void RemoveNode(Id node_id);

  /************************* SECT: DataLoad Manipulations *************************/

  /**
   * @brief Add a DataLoad to the graph.
   */
  Id AddDataLoad(std::unique_ptr<DataLoadBase> data_load);

  /**
   * @brief Remove a DataLoad from the graph.
   */
  void RemoveDataLoad(Id data_load_id);

  /************************* SECT: Connectivity *************************/
  /**
   * @brief Connect the output pin of a node to the input pin of another node through
   * one data load.
   */
  void Connect(Id from, Id to, Id data_load);

  /**
   * @brief Connect the output pin of a node to the input pin of another node, and
   * create the DataLoad automatically.
   */
  DataLoadBase* Connect(Id from, Id to);

  /**
   * @brief Check if the connection is valid.
   */
  bool CheckConnect(Pin* from, Pin* to);

  /**
   * @brief Disconnect the connection. return true if the connection exists and is
   * disconnected successfully.
   */
  bool Disconnect(Pin* from, Pin* to);

  /**
   * @brief Check if the connection exists.
   */
  bool HasConnection(Pin* from, Pin* to);

  /************************* SECT: Queries *************************/
  /**
   * @brief Get the node by its id.
   */
  NodeBase* GetNode(Id id);

  /**
   * @brief Get the DataLoad by its id.
   */
  DataLoadBase* GetDataLoad(Id id);

  /**
   * @brief Get all the nodes in the graph.
   */
  List<NodeBase*> GetNodes();

  /**
   * @brief Get all the nodes in the graph.
   */
  List<DataLoadBase*> GetDataLoads();

private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace ax::graph
