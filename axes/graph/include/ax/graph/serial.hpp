#pragma once
#include "graph.hpp"
#include <map>
#include <boost/json/object.hpp>

namespace ax::graph {

class Serializer {
public:
  Serializer(Graph& g);

  void SetNodeMetadata(idx node_id, boost::json::value const& obj);

  void SetSocketMetadata(idx socket_id, boost::json::value const& obj);

  boost::json::object Serialize() const;

private:
  Graph& graph_;

  std::map<idx, boost::json::value> node_metadata_;
  std::map<idx, boost::json::value> socket_metadata_;
};

class Deserializer {
public:
  Deserializer(Graph& g);

  Status Deserialize(boost::json::object const& obj);

  Graph& graph_;
  std::map<idx, idx> node_id_map_;
  std::map<idx, idx> inverse_node_id_map_;
  std::map<idx, idx> socket_id_map_;
  std::map<idx, idx> inverse_socket_id_map_;
  std::map<idx, boost::json::value> node_metadata_;
  std::map<idx, boost::json::value> socket_metadata_;
};

}