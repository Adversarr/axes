#include <imgui.h>
#include <imgui_node_editor.h>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/graph.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/io.hpp"
#include "ax/nodes/math_types.hpp"
#include "ax/nodes/stl_types.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/status.hpp"

using namespace ax;
using namespace ax::graph;
namespace ed = ax::NodeEditor;

void show_demo_window () {
  ImGui::ShowDemoWindow();
}

class IntToString : public NodeBase {
public:
  IntToString(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    auto pinput = RetriveInput<int>(0);
    std::string output;
    std::cout << "Running IntToString: " << pinput << std::endl;
    if (pinput == nullptr) {
      return utils::FailedPreconditionError("Input Pin 0 not connected.");
    } else {
      output = std::to_string(*pinput);
    }

    auto* p_put = RetriveOutput<std::string>(0);
    *p_put = output;
    AX_RETURN_OK();
  }
};

class IntInput : public NodeBase {
public:
  IntInput(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    *RetriveOutput<int>(0) = value_;
    AX_RETURN_OK();
  }

  int value_ = 0;
};

class StringOutput : public NodeBase {
public:
  StringOutput(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    auto* input = RetriveInput<std::string>(0);
    if (input == nullptr) {
      AX_LOG(INFO) << "Input not connected.";
    } else {
      AX_LOG(INFO) << "INPUT STRINGS IS: " << *input;
    }
    AX_RETURN_OK();
  }
};

class MeshAssetSelector : public NodeBase {
public:
  MeshAssetSelector(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    *RetriveOutput<std::string>(0) = selected_;
    AX_RETURN_OK();
  }

  Status OnConstruct() override {
    assets_ = utils::discover_assets("/mesh/obj/");
    selected_idx_ = 0;
    selected_ = assets_[0];
    AX_RETURN_OK();
  }

  std::string selected_;
  int selected_idx_;
  std::vector<std::string> assets_;
};

int main(int argc, char** argv) {
  gl::init(argc, argv);
  graph::install_renderer();

  NodeDescriptorFactory<MeshAssetSelector>{}
      .SetName("MeshAssetSelector")
      .SetDescription("Select mesh asset")
      .AddOutput<std::string>("path", "output description")
      .FinalizeAndRegister();

  nodes::register_stl_types();
  nodes::register_io_nodes();
  nodes::register_math_types_nodes();
  add_custom_node_render(
      typeid(MeshAssetSelector), CustomNodeRender{[](NodeBase* node) {
      ImGui::PushID(node);
        auto n = dynamic_cast<MeshAssetSelector*>(node);
        ImGui::SetNextItemWidth(100);
        if (ImGui::Button("Select!")) {
          ImGui::OpenPopup("Select Mesh");
        }

        ed::Suspend();
        if (ImGui::BeginPopup("Select Mesh")) {
          for (int i = 0; i < (idx)n->assets_.size(); ++i) {
            if (ImGui::Selectable(n->assets_[i].c_str(), n->selected_idx_ == i)) {
              n->selected_idx_ = i;
              n->selected_ = n->assets_[i];
            }
          }
          ImGui::EndPopup();
        }
        ed::Resume();

        ImGui::SameLine();
        ImGui::Dummy(ImVec2(30, 0));
        ImGui::SameLine();
        ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
        ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
        ed::EndPin();

        ImGui::Text("\"%s\"", n->selected_.c_str());

        ImGui::PopID();
      }});

  connect<gl::UiRenderEvent, &show_demo_window>();

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
