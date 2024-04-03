#include "ax/graph/common.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
class EmptyNode : public graph::NodeBase {
public:
  using graph::NodeBase::NodeBase;
  Status Run() { AX_RETURN_OK(); }
};

int main() {
  EmptyNode e(nullptr);
  auto [in] = e.RetriveInput<int>();
  auto [out] = e.RetriveOutput<int>();
}
