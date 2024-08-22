#pragma once
#include <imgui.h>
#include <imgui_node_editor.h>

#include <random>

#include "./all.hpp"
#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/common.hpp"
#include "ax/math/utils/formatting.hpp"

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

class Input_Color4 final : public NodeDerive<Input_Color4> {
public:
  CG_NODE_COMMON(Input_Color4, "Input/Color4", "Input a color");
  CG_NODE_INPUTS();

  CG_NODE_OUTPUTS((math::RealVector4, out, "The color"));

  static void RenderThis(NodeBase* ptr) {
    auto* this_ptr = static_cast<Input_Color4*>(ptr);
    begin_draw_node(ptr);
    draw_node_header_default(ptr);
    ImGui::ColorEdit4("Color", this_ptr->color_.data(),
                      ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_NoPicker
                          | ImGuiColorEditFlags_NoTooltip);
    draw_node_content_default(ptr);
    end_draw_node();
  }

  static void RenderWidget(NodeBase* ptr) {
    auto* this_ptr = static_cast<Input_Color4*>(ptr);
    ImGui::PushID(this_ptr);
    if (ImGui::ColorEdit4("Color", this_ptr->color_.data())) {
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
    obj["color"] = boost::json::array{color_.x(), color_.y(), color_.z(), color_.w()};
    return obj;
  }

  void Deserialize(boost::json::object const& obj) override try {
    NodeBase::Deserialize(obj);
    auto arr = obj.at("color").as_array();
    for (Index i = 0; i < 4; ++i) {
      color_[i] = static_cast<float>(arr.at(i).as_double());
    }
  } catch (std::exception const& e) {
    AX_ERROR("Failed to deserialize InputColor4: {}", e.what());
  }

  void operator()(Context&) override { SetAll(color_.cast<Real>()); }

  math::FloatVector4 color_ = math::FloatVector4::Zero();
};


#define DECLARE_INPUT_NODE(Type, deser, render_expr)                        \
  class Input_##Type final : public NodeDerive<Input_##Type> {              \
  public:                                                                   \
    CG_NODE_COMMON(Input_##Type, "Input/" #Type, "Input for " #Type);       \
    CG_NODE_INPUTS();                                                       \
    CG_NODE_OUTPUTS((math::Type, out, "The value"));                        \
    void SetOutput() { SetAll(value_); }                                    \
    void operator()(Context&) override { SetOutput(); }                     \
    static void RenderThis(NodeBase* ptr) {                                 \
      auto* this_ptr = static_cast<Input_##Type*>(ptr);                     \
      begin_draw_node(ptr);                                                 \
      draw_node_header_default(ptr);                                        \
      ImGui::Text("value=%s", fmt::format("{}", this_ptr->value_).c_str()); \
      draw_node_content_default(ptr);                                       \
      end_draw_node();                                                      \
    }                                                                       \
    static void RenderWidget(NodeBase* ptr) {                               \
      static const char* component_str[4] = {"X", "Y", "Z", "W"};           \
      auto* this_ptr = static_cast<Input_##Type*>(ptr);                     \
      ImGui::PushID(this_ptr);                                              \
      for (Index i = 0; i < this_ptr->value_.rows(); ++i) {                 \
        ImGui::PushID(&this_ptr->value_[i]);                                \
        render_expr;                                                        \
        ImGui::PopID();                                                     \
      }                                                                     \
      ImGui::PopID();                                                       \
    }                                                                       \
    static void OnRegister() {                                              \
      add_custom_node_render(name(), {RenderThis});                         \
      add_custom_node_widget(name(), {RenderWidget});                       \
    }                                                                       \
    math::Type value_ = math::Type::Zero();                                 \
    void Deserialize(boost::json::object const& obj) override try {         \
      auto array = obj.at("value").as_array();                              \
      for (Index i = 0; i < value_.rows(); ++i) {                           \
        value_[i] = array.at(i).deser();                                    \
      }                                                                     \
      SetOutput();                                                          \
    } catch (std::exception const& e) {                                     \
      AX_ERROR("Failed to deserialize Input_" #Type ": {}", e.what());      \
    }                                                                       \
    boost::json::object Serialize() const override {                        \
      auto obj = NodeBase::Serialize();                                     \
      auto array = boost::json::array{};                                    \
      for (Index i = 0; i < value_.rows(); ++i) {                           \
        array.push_back(value_[i]);                                         \
      }                                                                     \
      obj["value"] = array;                                                 \
      return obj;                                                           \
    }                                                                       \
  };

DECLARE_INPUT_NODE(IndexVector2, as_int64,
                   ImGui::InputScalar(component_str[i], ImGuiDataType_S64, &this_ptr->value_[i]));
DECLARE_INPUT_NODE(IndexVector3, as_int64,
                   ImGui::InputScalar(component_str[i], ImGuiDataType_S64, &this_ptr->value_[i]));
DECLARE_INPUT_NODE(IndexVector4, as_int64,
                   ImGui::InputScalar(component_str[i], ImGuiDataType_S64, &this_ptr->value_[i]));

DECLARE_INPUT_NODE(RealVector2, as_double, ImGui::InputDouble(component_str[i], &this_ptr->value_[i]));
DECLARE_INPUT_NODE(RealVector3, as_double, ImGui::InputDouble(component_str[i], &this_ptr->value_[i]));
DECLARE_INPUT_NODE(RealVector4, as_double, ImGui::InputDouble(component_str[i], &this_ptr->value_[i]));

void register_math_types(NodeRegistry& reg) {
  Input_Color3::RegisterTo(reg);
  Input_Color4::RegisterTo(reg);

  Input_IndexVector2::RegisterTo(reg);
  Input_IndexVector3::RegisterTo(reg);
  Input_IndexVector4::RegisterTo(reg);

  Input_RealVector2::RegisterTo(reg);
  Input_RealVector3::RegisterTo(reg);
  Input_RealVector4::RegisterTo(reg);
}
}  // namespace ax::nodes