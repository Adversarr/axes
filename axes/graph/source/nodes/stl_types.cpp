#include <imgui_node_editor.h>

#include "all.hpp"
#include "ax/core/entt.hpp"
#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/graph/common.hpp"
#include "ax/graph/render.hpp"

using namespace ax::graph;
namespace ed = ax::NodeEditor;

using namespace std;

namespace ax::nodes {

class Input_Index final : public NodeDerive<Input_Index> {
public:
  CG_NODE_COMMON(Input_Index, "Input/Index", "ImGui Input integer");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((int, value, "value from UI input."));

  static void RenderThis(NodeBase* ptr) {
    begin_draw_node(ptr);
    draw_node_header_default(ptr);
    ImGui::SetNextItemWidth(100);
    auto* p = static_cast<Input_Index*>(ptr);
    ImGui::PushID(p);
    if (ImGui::InputScalar("value", ImGuiDataType_S64, &p->value_)) {
      p->SetAll(p->value_);
    }
    ImGui::PopID();
    draw_node_content_default(ptr);
    end_draw_node();
  }

  static void OnRegister() { add_custom_node_render(name(), {RenderThis}); }

  boost::json::object Serialize() const override { return {{"Index", value_}}; }

  void Deserialize(boost::json::object const& from) override try {
    value_ = from.at("Index").as_int64();
    SetAll(value_);
  } catch (std::exception const& e) {
    AX_ERROR("Failed to deserialize Input_Index: {}", e.what());
  }

  void operator()(Context&) override { SetAll(value_); }

private:
  Index value_ = 0;
};

class Input_Real final : public NodeDerive<Input_Real> {
public:
  CG_NODE_COMMON(Input_Real, "Input/Real", "ImGui Input Real");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((float, value, "value from UI input."));

  static void RenderThis(NodeBase* ptr) {
    begin_draw_node(ptr);
    draw_node_header_default(ptr);
    ImGui::SetNextItemWidth(100);
    auto* p = static_cast<Input_Real*>(ptr);
    if (ImGui::InputScalar("value", ImGuiDataType_Double, &p->value_)) {
      p->SetAll(p->value_);
    }
    draw_node_content_default(ptr);
    end_draw_node();
  }

  static void OnRegister() { add_custom_node_render(name(), {RenderThis}); }

  boost::json::object Serialize() const override { return {{"value", value_}}; }

  void Deserialize(boost::json::object const& from) override try {
    value_ = from.at("value").as_double();
    SetAll(value_);
  } catch (std::exception const& e) {
    AX_ERROR("Failed to deserialize Input_Real: {}", e.what());
  }

  void operator()(Context&) override { SetAll(value_); }

private:
  Real value_ = 0.0f;
};

class Input_string final : public NodeDerive<Input_string> {
public:
  CG_NODE_COMMON(Input_string, "Input/string", "ImGui Input string");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((std::string, value, "value from UI input."));

  static void RenderThis(NodeBase* ptr) {
    begin_draw_node(ptr);
    draw_node_header_default(ptr);
    ImGui::SetNextItemWidth(200);
    auto* p = static_cast<Input_string*>(ptr);
    ImGui::SetNextItemWidth(200);
    ImGui::TextUnformatted(p->value_);
    draw_node_content_default(ptr);
    end_draw_node();
  }

  static void RenderWidget(NodeBase* ptr) {
    auto* p = static_cast<Input_string*>(ptr);
    if (ImGui::InputTextMultiline("value", p->value_, sizeof(p->value_))) {
      p->value_[sizeof(p->value_) - 1] = '\0';
      p->SetAll(p->value_);
    }
  }

  boost::json::object Serialize() const override { return {{"value", value_}}; }

  void Deserialize(boost::json::object const& from) override try {
    auto const& value = from.at("value").as_string();
    strncpy(value_, value.c_str(), sizeof(value_));
    SetAll(value_);
  } catch (std::exception const& e) {
    AX_ERROR("Failed to deserialize Input_string: {}", e.what());
  }

  static void OnRegister() {
    add_custom_node_render(name(), {RenderThis});
    add_custom_node_widget(name(), {RenderWidget});
  }

  void operator()(Context&) override { SetAll(value_); }

private:
  char value_[256] = "";
};

class Input_bool final : public NodeDerive<Input_bool> {
public:
  CG_NODE_COMMON(Input_bool, "Input/bool", "ImGui Input boolean");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((bool, value, "value from UI input."));

  static void RenderThis(NodeBase* ptr) {
    begin_draw_node(ptr);
    draw_node_header_default(ptr);
    auto* p = static_cast<Input_bool*>(ptr);
    ImGui::SetNextItemWidth(200);
    if (ImGui::Checkbox("value", &p->value_)) {
      p->SetAll(p->value_);
    }
    draw_node_content_default(ptr);
    end_draw_node();
  }

  static void OnRegister() { add_custom_node_render(name(), {RenderThis}); }

  boost::json::object Serialize() const override { return {{"value", value_}}; }

  void Deserialize(boost::json::object const& from) override try {
    value_ = from.at("value").as_bool();
    SetAll(value_);
  } catch (std::exception const& e) {
    AX_ERROR("Failed to deserialize Input_bool: {}", e.what());
  }

  void operator()(Context&) override { SetAll(value_); }

private:
  bool value_ = false;
};

void register_stl_types(NodeRegistry& reg) {
  Input_Index::RegisterTo(reg);
  Input_Real::RegisterTo(reg);
  Input_bool::RegisterTo(reg);
  Input_string::RegisterTo(reg);
}

}  // namespace ax::nodes
