#include "ax/nodes/io.hpp"

#include "ax/geometry/common.hpp"
#include "ax/geometry/io.hpp"
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
        .SetName("Read_obj")
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

    bool need_reload_this_frame;
    if (reload == nullptr) {
      need_reload_this_frame = frame_id == 0;
    } else {
      need_reload_this_frame = *reload;
    }

    if (need_reload_this_frame) {
      auto mesh = geo::read_obj(*file);
      if (!mesh.ok()) {
        return mesh.status();
      }
      *RetriveOutput<geo::SurfaceMesh>(0) = std::move(mesh.value());
    }

    AX_RETURN_OK();
  }
};


void register_io_nodes() {
  ReadObjNode::register_this();
}
}  // namespace ax::nodes
