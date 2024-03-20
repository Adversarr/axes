#include "axes/graph/common.hpp"

#include "axes/utils/status.hpp"


using namespace ax;
class EmptyNode : public graph::NodeBase {
public:
  Status Run() { AX_RETURN_OK(); }
};

int main() { 
  EmptyNode e;
  int x;
  auto result = e.RetriveInput<int>();
}