#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/graph.hpp"
#include "ax/graph/node.hpp"
#include "ax/utils/status.hpp"
#include "ax/graph/render.hpp"
#include <imgui.h>
#include <imgui_node_editor.h>

using namespace ax;
using namespace ax::graph;
namespace ed = ax::NodeEditor;

class IntToString : public NodeBase {
public:
  IntToString(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    AX_RETURN_OK();
  }
};

class IntInput : public NodeBase {
public:
  IntInput(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    AX_RETURN_OK();
  }

  int value_;
};

class StringOutput : public NodeBase {
public:
  StringOutput(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  Status Apply(idx) override {
    AX_RETURN_OK();
  }
};



int main(int argc, char** argv) {
  gl::init(argc, argv);
  graph::install_renderer();
  auto node_desc = NodeDescriptorFactory<IntToString>{}
        .SetName("IntToString")
        .SetDescription("Convert int to string")
        .AddInput(PinDescriptor{typeid(int), "input", "input description"})
        .AddOutput(PinDescriptor{typeid(std::string), "output", "output description"})
        .Finalize();

  auto node_desc1 = NodeDescriptorFactory<IntInput>{}
        .SetName("IntInput")
        .SetDescription("Input string")
        .AddOutput(PinDescriptor{typeid(int), "output", "output description"})
        .Finalize();

  auto node_desc2 = NodeDescriptorFactory<StringOutput>{}
        .SetName("StringOutput")
        .SetDescription("Output string")
        .AddInput(PinDescriptor{typeid(std::string), "input", "input description"})
        .Finalize();

  details::factory_register(node_desc);
  details::factory_register(node_desc1);
  details::factory_register(node_desc2);

  add_custem_node_render(typeid(IntInput), CustomNodeRender{[](NodeBase* node) {
    auto n = dynamic_cast<IntInput*>(node);
    ImGui::SetNextItemWidth(100);
    ImGui::InputInt("input", &n->value_);
    ImGui::SameLine();
    ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
    ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
    ed::EndPin();
  }});

  AX_CHECK_OK(gl::enter_main_loop());
  clean_up();
  return 0;
}
