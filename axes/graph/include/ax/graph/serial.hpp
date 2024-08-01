#pragma once
#include "graph.hpp"
#include <map>
#include <boost/json/object.hpp>

namespace ax::graph {

class Serializer {
public:
  Serializer(Graph& g);

  void SetNodeMetadata(ident_t node_id, boost::json::value const& obj);

  void SetSocketMetadata(ident_t socket_id, boost::json::value const& obj);

  boost::json::object Serialize() const;

private:
  Graph& graph_;

  std::map<ident_t, boost::json::value> node_metadata_;
  std::map<ident_t, boost::json::value> socket_metadata_;
};

class Deserializer {
public:
  Deserializer(Graph& g);

  void Deserialize(boost::json::object const& obj);

  Graph& graph_;
  std::map<ident_t, ident_t> node_id_map_;
  std::map<ident_t, ident_t> inverse_node_id_map_;
  std::map<ident_t, ident_t> socket_id_map_;
  std::map<ident_t, ident_t> inverse_socket_id_map_;
  std::map<ident_t, boost::json::value> node_metadata_;
  std::map<ident_t, boost::json::value> socket_metadata_;
};

}
