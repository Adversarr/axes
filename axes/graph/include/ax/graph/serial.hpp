#pragma once
#include "graph.hpp"
#include <map>
#include <boost/json/object.hpp>

namespace ax::graph {

class Serializer {
public:
  Serializer(Graph& g);

  void SetNodeMetadata(id_t node_id, boost::json::value const& obj);

  void SetSocketMetadata(id_t socket_id, boost::json::value const& obj);

  boost::json::object Serialize() const;

private:
  Graph& graph_;

  std::map<id_t, boost::json::value> node_metadata_;
  std::map<id_t, boost::json::value> socket_metadata_;
};

class Deserializer {
public:
  Deserializer(Graph& g);

  void Deserialize(boost::json::object const& obj);

  Graph& graph_;
  std::map<id_t, id_t> node_id_map_;
  std::map<id_t, id_t> inverse_node_id_map_;
  std::map<id_t, id_t> socket_id_map_;
  std::map<id_t, id_t> inverse_socket_id_map_;
  std::map<id_t, boost::json::value> node_metadata_;
  std::map<id_t, boost::json::value> socket_metadata_;
};

}
