#include <doctest/doctest.h>
#include "ax/graph/graph.hpp"
#include "ax/core/init.hpp"
#include "ax/graph/node.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
using namespace ax::graph;

class Node : public NodeBase {
public:
  Node(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx frame_id) override {
    AX_RETURN_OK();
  }
};

TEST_CASE("Graph") {
  ax::init();
  auto node_desc = NodeDescriptorFactory<Node>{}
          .SetName("Node")
          .SetDescription("Node description")
          .AddInput(PinDescriptor{typeid(int), "input", "input description"})
          .AddOutput(PinDescriptor{typeid(int), "output", "output description"})
          .AddOutput(PinDescriptor{typeid(std::string), "str out", "output description"})
          .Finalize();

  details::factory_register(node_desc);
  Graph graph;
  auto node1 = graph.AddNode(&node_desc);
  auto node2 = graph.AddNode(&node_desc);
  CHECK(graph.CanConnectSocket(node1->GetId(), 0, node2->GetId(), 0));
  CHECK(!graph.CanConnectSocket(node1->GetId(), 1, node2->GetId(), 0));

  auto sock = graph.AddSocket(node1->GetId(), 0, node2->GetId(), 0);
  auto sock2 = graph.GetSocket(node1->GetId(), 0, node2->GetId(), 0);
  CHECK(sock != nullptr);
  CHECK(sock == sock2);
}
