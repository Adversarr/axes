#include <doctest/doctest.h>
#include "ax/graph/graph.hpp"
#include "ax/core/init.hpp"
#include "ax/graph/node.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
using namespace ax::graph;

class IntToString : public NodeBase {
public:
  IntToString(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    AX_RETURN_OK();
  }
};

TEST_CASE("Graph") {
  ax::init();
  auto node_desc = NodeDescriptorFactory<IntToString>{}
          .SetName("Node")
          .SetDescription("Node description")
          .AddInput<int>("in", "input description")
          .AddOutput<int>("out", "output description")
          .AddOutput<int>("str out", "output description")
          .Finalize();

  details::factory_register(node_desc);
  Graph graph;
  auto node1 = graph.AddNode(&node_desc).value();
  auto node2 = graph.AddNode(&node_desc).value();
  CHECK(graph.CanConnectSocket(node1->GetId(), 0, node2->GetId(), 0));
  CHECK(!graph.CanConnectSocket(node1->GetId(), 1, node2->GetId(), 0));

  auto sock = graph.AddSocket(node1->GetId(), 0, node2->GetId(), 0);
  auto sock2 = graph.GetSocket(node1->GetId(), 0, node2->GetId(), 0);
  CHECK(sock.value() != nullptr);
  CHECK(sock.value() == sock2);

  graph.RemoveSocket(node1->GetId(), 0, node2->GetId(), 0);
  CHECK(graph.GetSocket(node1->GetId(), 0, node2->GetId(), 0) == nullptr);

  sock = graph.AddSocket(node1->GetId(), 0, node2->GetId(), 0);
  CHECK(sock.value() != nullptr);
  CHECK(graph.GetSocket(node1->GetId(), 0, node2->GetId(), 0) != nullptr);

  graph.RemoveNode(node1->GetId());
  CHECK(graph.GetNode(node1->GetId()) == nullptr);
  CHECK(graph.GetSocket(node1->GetId(), 0, node2->GetId(), 0) == nullptr);

  graph.Clear();
  std::vector<idx> nodes;
  std::vector<Socket* > sockets;
  for (int i = 0; i < 10; i++) {
    nodes.push_back(graph.AddNode(&node_desc).value()->GetId());
  }
  CHECK(graph.GetNumNodes() == 10);
  CHECK(graph.GetNumSockets() == 0);
  for (int i = 0; i < 9; ++i) {
    sockets.push_back(graph.AddSocket(nodes[i], 0, nodes[i + 1], 0).value());
  }

  // Erase 5, and 4-5, 5-6 should not exist.
  CHECK(graph.RemoveNode(nodes[5]));
  CHECK(graph.GetNumNodes() == 9);
  CHECK(graph.GetNumSockets() == 7);
  CHECK(graph.GetSocket(nodes[4], 0, nodes[5], 0) == nullptr);
  CHECK(graph.GetSocket(nodes[5], 0, nodes[6], 0) == nullptr);
}
