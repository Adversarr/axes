//
// Created by adversarr on 4/13/24.
//
#include "ax/nodes/stl_types.hpp"

#include <imnode/imgui_node_editor.h>

#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/utils/status.hpp"

using namespace ax::graph;
namespace ed = ax::NodeEditor;

using namespace std;

namespace ax::nodes {

#define DefineInputNodeBegin(class_name)                                                   \
  class Input_##class_name : public NodeBase {                                             \
  public:                                                                                  \
    Input_##class_name(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {} \
    static void register_this() {                                                          \
      NodeDescriptorFactory<Input_##class_name>{}                                          \
          .SetName("Input_" #class_name)                                                   \
          .AddOutput<class_name>("value", "value from input")                              \
          .FinalizeAndRegister();                                                          \
      add_custom_node_render(typeid(Input_##class_name), CustomNodeRender{[](NodeBase* n) {       \
auto node = dynamic_cast<Input_##class_name *>(n);
#define DefineInputNodeEnd() }});}}

DefineInputNodeBegin(int)
    auto* p_put = node->RetriveOutput<int>(0);
    ImGui::SetNextItemWidth(100);
    ImGui::PushID(p_put);
    ImGui::InputInt("value", p_put);
    ImGui::PopID();
    ImGui::SameLine();
    ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
    ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
    ed::EndPin();
DefineInputNodeEnd();

DefineInputNodeBegin(float)
    auto* p_put = node->RetriveOutput<float>(0);
    ImGui::SetNextItemWidth(100);
    ImGui::PushID(p_put);
    ImGui::InputFloat("value", p_put);
    ImGui::PopID();
    ImGui::SameLine();
    ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
    ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
    ed::EndPin();
DefineInputNodeEnd();

DefineInputNodeBegin(idx)
    auto* p_put = node->RetriveOutput<idx>(0);
    ImGui::SetNextItemWidth(100);
    ImGui::PushID(p_put);
    ImGui::InputScalar("value", ImGuiDataType_S64, p_put);
    ImGui::PopID();
    ImGui::SameLine();
    ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
    ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
    ed::EndPin();
DefineInputNodeEnd();

DefineInputNodeBegin(string)
    auto* p_put = node->RetriveOutput<std::string>(0);
    ImGui::SetNextItemWidth(100);
    ImGui::PushID(p_put);
    if (ImGui::InputText("value", node->buffer, 256, ImGuiInputTextFlags_EnterReturnsTrue)) {
      *p_put = node->buffer;
    }
    ImGui::PopID();
    ImGui::SameLine();
    ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
    ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
    ed::EndPin();
  }});}
private:
    char buffer[256] {0};
};

void register_stl_types() {
  Input_int::register_this();
  Input_float::register_this();
  Input_idx::register_this();
  Input_string::register_this();
}

}  // namespace ax::nodes