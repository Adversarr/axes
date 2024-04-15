#include "ax/nodes/io.hpp"

#include "ax/geometry/common.hpp"
#include "ax/geometry/io.hpp"
#include "ax/utils/asset.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/utils/status.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>

namespace ed = ax::NodeEditor;
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
        AX_LOG(ERROR) << "Failed to read obj file: " << file->c_str();
        return mesh.status();
      }
      *RetriveOutput<geo::SurfaceMesh>(0) = std::move(mesh.value());
    }

    AX_RETURN_OK();
  }
};

class Selector_Mesh_Obj : public NodeBase {
public:

  Selector_Mesh_Obj(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Selector_Mesh_Obj>()
        .SetName("Selector_mesh_obj")
        .SetDescription("Select an asset from a list")
        .AddOutput<std::string>("mesh", "The mesh read from the obj file")
        .FinalizeAndRegister();

    add_custom_node_render(typeid(Selector_Mesh_Obj), CustomNodeRender{[](NodeBase* node) {
        auto n = dynamic_cast<Selector_Mesh_Obj*>(node);
        ImGui::SetNextItemWidth(100);
        if (ImGui::Button("Select")) {
          ImGui::OpenPopup("Select Mesh");
        }

        ImGui::SameLine();
        ImGui::SameLine();
        ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
        ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
        ed::EndPin();
        ImGui::Text("\"%s\"", n->assets_[n->selected_idx_].c_str());

        ed::Suspend();
        if (ImGui::BeginPopup("Select Mesh")) {
          for (int i = 0; i < (idx)n->assets_.size(); ++i) {
            if (ImGui::Selectable(n->assets_[i].c_str(), n->selected_idx_ == i)) {
              n->selected_idx_ = i;
            }
          }
          ImGui::EndPopup();
        }
        ed::Resume();
      }});
  }

  Status PreCompute() override {
    *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_idx_]);
    AX_RETURN_OK();
  }

  Status OnConstruct() override {
    assets_ = utils::discover_assets("/mesh/obj/");
    selected_idx_ = 0;
    AX_RETURN_OK();
  }

  std::vector<std::string> assets_;
  int selected_idx_;
};


void register_io_nodes() {
  ReadObjNode::register_this();
  Selector_Mesh_Obj::register_this();
}
}  // namespace ax::nodes
