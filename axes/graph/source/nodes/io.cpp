#include "ax/nodes/io.hpp"
#include "ax/geometry/common.hpp"
#include "ax/graph/node.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
using namespace graph;

namespace ax::nodes {

class ReadObjNode : public NodeBase {
public:
    ReadObjNode(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
        NodeDescriptorFactory<ReadObjNode>()
            .SetName("ReadObj")
            .SetDescription("Reads an obj file and outputs a mesh")
            .AddInput<std::string>("file", "The path to the obj file")
            .AddInput<bool>("Reload Every Frame?", "If true, the obj file will be reloaded every frame")
            .AddOutput<geo::SurfaceMesh>("mesh", "The mesh read from the obj file")
            .FinalizeAndRegister();
    }

    Status Apply(idx frame_id) {
      auto* file = RetriveInput<std::string>(0);
      auto* reload = RetriveInput<bool>(1);
      if (file == nullptr) {
        return utils::FailedPreconditionError("File path is not set");
      }
    }
};
}