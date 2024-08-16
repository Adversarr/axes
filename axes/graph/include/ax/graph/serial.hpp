#pragma once
#include <boost/json/object.hpp>
#include <map>

#include "common.hpp"

namespace ax::graph {

class Serializer {
public:
  explicit Serializer(Graph& g);
  void SetNodeMetadata(ConstNodePtr n, boost::json::object const& obj);

  boost::json::object Serialize() const;
  void Deserialize(boost::json::object const& obj);

  auto const& GetNodeMetadata() const { return node_metadata_; }

private:
  Graph& graph_;
  std::map<ConstNodePtr, boost::json::object> node_metadata_;
};
}  // namespace ax::graph
