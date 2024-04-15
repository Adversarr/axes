#include "ax/nodes/math_types.hpp"
#include <imgui.h>
#include <imgui_node_editor.h>

#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/common.hpp"
#include "ax/utils/status.hpp"

namespace ed = ax::NodeEditor;

namespace ax::nodes {
using namespace graph;

class InputColor3f : public NodeBase {
  public:
    InputColor3f(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<InputColor3f>()
          .SetName("Input_color3f")
          .SetDescription("Input a color")
          .AddOutput<math::vec3r>("out", "The color")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(InputColor3f), {[](NodeBase* node) {
          auto n = static_cast<InputColor3f*>(node);
          ImVec4 color(n->color_.x(), n->color_.y(), n->color_.z(), 1.0f);
          ImGui::SetNextItemWidth(100);
          if (ImGui::ColorButton("Color Picker", color)) {
            ImGui::OpenPopup("ColorPicker");
          }
          ed::Suspend();
          if (ImGui::BeginPopup("ColorPicker")) {
            ImGui::ColorPicker3("Color", n->color_.data(), ImGuiColorEditFlags_Float);
            ImGui::EndPopup();
          }
          ed::Resume();
          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
      }});
    }

    Status Apply(idx) {
      SetOutput<math::vec3r>(0, color_.cast<real>());
      AX_RETURN_OK();
    }

    math::vec3f color_;
};

class InputColor4f : public NodeBase {
  public:
    InputColor4f(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<InputColor4f>()
          .SetName("Input_color4f")
          .SetDescription("Input a color")
          .AddOutput<math::vec4r>("out", "The color")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(InputColor4f), {[](NodeBase* node) {
          auto n = static_cast<InputColor4f*>(node);
          ImVec4 color(n->color_.x(), n->color_.y(), n->color_.z(), n->color_.w());
          ImGui::SetNextItemWidth(100);
          if (ImGui::ColorButton("Color Picker", color)) {
            ImGui::OpenPopup("ColorPicker");
          }

          ed::Suspend();
          if (ImGui::BeginPopup("ColorPicker")) {
            ImGui::ColorPicker4("Color", n->color_.data(), ImGuiColorEditFlags_Float);
            ImGui::EndPopup();
          }
          ed::Resume();

          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
      }});
    }

    Status Apply(idx) {
      SetOutput<math::vec4r>(0, color_.cast<real>());
      AX_RETURN_OK();
    }
    math::vec4f color_;
};

class Input_Vec2r : public NodeBase {
  public:
    Input_Vec2r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<Input_Vec2r>()
          .SetName("Input_Vec2r")
          .SetDescription("Input a 2D vector")
          .AddOutput<math::vec2r>("vector", "The vector")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(Input_Vec2r), {[](NodeBase* node) {
          auto n = static_cast<Input_Vec2r*>(node);
          auto *p = n->RetriveOutput<math::vec2r>(0);
          ImGui::SetNextItemWidth(100);
          ImGui::InputDouble("X", &(p->x()));
          ImGui::SetNextItemWidth(100);
          ImGui::InputDouble("Y", &(p->y()));
          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
      }});
    }
};

class Input3f : public NodeBase {
  public:
    Input3f(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<Input3f>()
          .SetName("Input_Vec3r")
          .SetDescription("Input a 3D vector")
          .AddOutput<math::vec3r>("out", "The vector")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(Input3f), {[](NodeBase* node) {
          auto n = static_cast<Input3f*>(node);
          ImGui::SetNextItemWidth(300);
          ImGui::InputFloat3("Vector", n->vector_.data());
          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
      }});
    }

    Status Apply(idx) {
      SetOutput<math::vec3r>(0, vector_.cast<real>());
      AX_RETURN_OK();
    }

  private:
    math::vec3f vector_;
};

class Input4f : public NodeBase {
  public:
    Input4f(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<Input4f>()
          .SetName("Input_4f")
          .SetDescription("Input a 4D vector")
          .AddOutput<math::vec4r>("out", "The vector")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(Input4f), {[](NodeBase* node) {
          auto n = static_cast<Input4f*>(node);
          ImGui::SetNextItemWidth(400);
          ImGui::InputFloat4("Vector", n->vector_.data());

          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s ->", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
      }});
    }

    Status Apply(idx) {
      SetOutput<math::vec4r>(0, vector_.cast<real>());
      AX_RETURN_OK();
    }

  private:
    math::vec4f vector_;
};

template<idx dim>
class RealVectorGetIndex : public NodeBase {
  public:
    RealVectorGetIndex(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<RealVectorGetIndex<dim>>()
          .SetName("Vec" + std::to_string(dim) + "r_get_index")
          .SetDescription("Get a component of a vector")
          .template AddInput<math::vecr<dim>>("vec", "The vector")
          .template AddInput<idx>("index", "The index of the component to get")
          .template AddOutput<real>("out", "The component")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      auto* vector = RetriveInput<math::vecr<dim>>(0);
      auto* index = RetriveInput<idx>(1);
      if (vector == nullptr) {
        return utils::FailedPreconditionError("Vector is not set");
      }
      if (index == nullptr) {
        return utils::FailedPreconditionError("Index is not set");
      }
      if (*index < 0 || *index >= dim) {
        return utils::FailedPreconditionError("Index out of bounds");
      }
      SetOutput<real>(0, (*vector)[*index]);
      AX_RETURN_OK();
    }
};


void register_math_types_nodes() {
  InputColor3f::register_this();
  InputColor4f::register_this();
  Input_Vec2r::register_this();
  Input3f::register_this();
  Input4f::register_this();

  RealVectorGetIndex<2>::register_this();
  RealVectorGetIndex<3>::register_this();
  RealVectorGetIndex<4>::register_this();
}

}  // namespace ax::nodes
