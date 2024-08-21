#pragma once
#include <imgui.h>
#include <imgui_node_editor.h>

#include <random>

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/common.hpp"
#include "ax/math/functional.hpp"

#include "./all.hpp"

namespace ed = ax::NodeEditor;
using namespace ax::graph;

namespace ax::nodes {

class Input_Color3 final : public NodeDerive<Input_Color3> {
public:
  CG_NODE_COMMON(Input_Color3, "Input/Color3", "Input a color");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((math::RealVector3, out, "The color"));

  static void RenderThis(NodeBase* ptr) {
    auto* this_ptr = static_cast<Input_Color3*>(ptr);
    begin_draw_node(ptr);
    draw_node_header_default(ptr);
    ImGui::ColorEdit3("Color", this_ptr->color_.data(),
                      ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoPicker
                          | ImGuiColorEditFlags_NoTooltip);
    draw_node_content_default(ptr);
    end_draw_node();
  }

  static void RenderWidget(NodeBase* ptr) {
    auto* this_ptr = static_cast<Input_Color3*>(ptr);
    ImGui::PushID(this_ptr);
    if (ImGui::ColorEdit3("Color", this_ptr->color_.data())) {
      this_ptr->SetAll(this_ptr->color_.cast<Real>());
    }
    ImGui::PopID();
  }

  static void OnRegister() {
    add_custom_node_render(name(), {RenderThis});
    add_custom_node_widget(name(), {RenderWidget});
  }

  boost::json::object Serialize() const override {
    auto obj = NodeBase::Serialize();
    obj["color"] = boost::json::array{color_.x(), color_.y(), color_.z()};
    return obj;
  }

  void Deserialize(boost::json::object const& obj) override try {
    NodeBase::Deserialize(obj);
    auto arr = obj.at("color").as_array();
    for (Index i = 0; i < 3; ++i) {
      color_[i] = static_cast<float>(arr.at(i).as_double());
    }
  } catch (std::exception const& e) {
    AX_ERROR("Failed to deserialize InputColor3: {}", e.what());
  }

  void operator()(Context&) override { SetAll(color_.cast<Real>()); }

  math::FloatVector3 color_ = math::FloatVector3::Zero();
};

void register_math_types(NodeRegistry& reg) {
  Input_Color3::RegisterTo(reg);
}

}  // namespace ax::nodes
