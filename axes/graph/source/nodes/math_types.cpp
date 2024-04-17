#include "ax/nodes/math_types.hpp"
#include <imgui.h>
#include <imgui_node_editor.h>
#include <random>

#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/common.hpp"
#include "ax/math/functional.hpp"
#include "ax/utils/status.hpp"

namespace ed = ax::NodeEditor;

namespace ax::nodes {
using namespace graph;

const char names [4][2]  = {
  "x", "y", "z", "w"
};

class InputColor3r : public NodeBase {
  public:
    InputColor3r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<InputColor3r>()
          .SetName("Input_color3r")
          .SetDescription("Input a color")
          .AddOutput<math::vec3r>("out", "The color")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(InputColor3r), {[](NodeBase* node) {
        begin_draw_node(node);draw_node_header_default(node);
        auto n = static_cast<InputColor3r*>(node);
        ImVec4 color(n->color_.x(), n->color_.y(), n->color_.z(), 1.0f);
        ImGui::SetNextItemWidth(100);
        if (ImGui::ColorButton("Color Picker", color)) {
          ImGui::OpenPopup("ColorPicker");
        }
        ImGui::SameLine();
        ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
        ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
        ed::EndPin();
        end_draw_node();

        ed::Suspend();
        ImGui::PushID(node);
        if (ImGui::BeginPopup("ColorPicker")) {
          ImGui::ColorPicker3("Color", n->color_.data(), ImGuiColorEditFlags_Float);
          ImGui::EndPopup();
        }
        ImGui::PopID();
        ed::Resume();
      }});
    }

    Status PreApply() final {
      SetOutput<math::vec3r>(0, color_.cast<real>());
      AX_RETURN_OK();
    }

    boost::json::object Serialize() const override {
      auto obj = NodeBase::Serialize();
      obj["color"] = boost::json::array{color_.x(), color_.y(), color_.z()};
      return obj;
    }

    void Deserialize(boost::json::object const& obj) override {
      NodeBase::Deserialize(obj);
      if (!obj.contains("color")) {
        return;
      }
      auto arr = obj.at("color").as_array();
      color_ = math::vec3f(arr[0].as_double(), arr[1].as_double(), arr[2].as_double());
    }

    math::vec3f color_;
};

class InputColor4r : public NodeBase {
  public:
    InputColor4r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<InputColor4r>()
          .SetName("Input_color4r")
          .SetDescription("Input a color")
          .AddOutput<math::vec4r>("out", "The color")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(InputColor4r), {[](NodeBase* node) {
        begin_draw_node(node);draw_node_header_default(node);
          auto n = static_cast<InputColor4r*>(node);
          ImVec4 color(n->color_.x(), n->color_.y(), n->color_.z(), n->color_.w());
          ImGui::SetNextItemWidth(100);
          if (ImGui::ColorButton("Color Picker", color)) {
            ImGui::OpenPopup("ColorPicker");
          }


          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
        end_draw_node();

        ed::Suspend();
        ImGui::PushID(node);
        if (ImGui::BeginPopup("ColorPicker")) {
          ImGui::ColorPicker4("Color", n->color_.data(), ImGuiColorEditFlags_Float);
          ImGui::EndPopup();
        }
        ImGui::PopID();
        ed::Resume();
      }});
    }

    Status Apply(idx) final {
      SetOutput<math::vec4r>(0, color_.cast<real>());
      AX_RETURN_OK();
    }

    boost::json::object Serialize() const override {
      auto obj = NodeBase::Serialize();
      obj["color"] = boost::json::array{color_.x(), color_.y(), color_.z(), color_.w()};
      return obj;
    }

    void Deserialize(boost::json::object const& obj) override {
      NodeBase::Deserialize(obj);
      if (!obj.contains("color")) {
        return;
      }
      auto arr = obj.at("color").as_array();
      color_ = math::vec4f(arr[0].as_double(), arr[1].as_double(), arr[2].as_double(), arr[3].as_double());
    }

    math::vec4f color_;
};

class Input_Vec2r : public NodeBase {
  public:
    Input_Vec2r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<Input_Vec2r>()
          .SetName("Input_vec2r")
          .SetDescription("Input a 2D vector")
          .AddOutput<math::vec2r>("vector", "The vector")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(Input_Vec2r), {[](NodeBase* node) {
        begin_draw_node(node);draw_node_header_default(node);
          auto n = static_cast<Input_Vec2r*>(node);
          auto *p_out = n->RetriveOutput<math::vec2r>(0);
          for (int i = 0; i < 2; i++) {
            ImGui::SetNextItemWidth(100);
            ImGui::InputDouble(names[i], p_out->data() + i);
          }
          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
        end_draw_node();
      }});
    }

    boost::json::object Serialize() const override {
      auto obj = NodeBase::Serialize();
      auto* p_out = RetriveOutput<math::vec2r>(0);
      obj["x"] = p_out->x();
      obj["y"] = p_out->y();
      return obj;
    }

    void Deserialize(boost::json::object const& obj) override {
      NodeBase::Deserialize(obj);
      auto* p_out = RetriveOutput<math::vec2r>(0);
      if (obj.contains("x")) {
        p_out->x() = obj.at("x").as_double();
      }
      if (obj.contains("y")) {
        p_out->y() = obj.at("y").as_double();
      }
    }

};

class Input_Vec3r : public NodeBase {
  public:
    Input_Vec3r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<Input_Vec3r>()
          .SetName("Input_vec3r")
          .SetDescription("Input a 3D vector")
          .AddOutput<math::vec3r>("out", "The vector")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(Input_Vec3r), {[](NodeBase* node) {
        begin_draw_node(node);draw_node_header_default(node);
          auto n = static_cast<Input_Vec3r*>(node);
          auto* p_out = n->RetriveOutput<math::vec3r>(0);
          for (int i = 0; i < 3; i++) {
            ImGui::SetNextItemWidth(100);
            ImGui::InputDouble(names[i], p_out->data() + i);
          }
          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
        end_draw_node();
      }});
    }

    boost::json::object Serialize() const override {
      auto obj = NodeBase::Serialize();
      auto* p_out = RetriveOutput<math::vec3r>(0);
      obj["x"] = p_out->x();
      obj["y"] = p_out->y();
      obj["z"] = p_out->z();
      return obj;
    }

    void Deserialize(boost::json::object const& obj) override {
      NodeBase::Deserialize(obj);
      auto* p_out = RetriveOutput<math::vec3r>(0);
      if (obj.contains("x")) {
        p_out->x() = obj.at("x").as_double();
      }
      if (obj.contains("y")) {
        p_out->y() = obj.at("y").as_double();
      }
      if (obj.contains("z")) {
        p_out->z() = obj.at("z").as_double();
      }
    }

};

class Input_Vec4r : public NodeBase {
  public:
    Input_Vec4r(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<Input_Vec4r>()
          .SetName("Input_vec4r")
          .SetDescription("Input a 4D vector")
          .AddOutput<math::vec4r>("out", "The vector")
          .FinalizeAndRegister();

      add_custom_node_render(typeid(Input_Vec4r), {[](NodeBase* node) {
        begin_draw_node(node);draw_node_header_default(node);
          auto n = static_cast<Input_Vec4r*>(node);
          auto* p_out = n->RetriveOutput<math::vec4r>(0);
          for (int i = 0; i < 4; i++) {
            ImGui::SetNextItemWidth(100);
            ImGui::InputDouble(names[i], p_out->data() + i);
          }
          ImGui::SameLine();
          ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
        end_draw_node();
      }});
    }

    boost::json::object Serialize() const override {
      auto obj = NodeBase::Serialize();
      auto* p_out = RetriveOutput<math::vec4r>(0);
      obj["x"] = p_out->x();
      obj["y"] = p_out->y();
      obj["z"] = p_out->z();
      obj["w"] = p_out->w();
      return obj;
    }

    void Deserialize(boost::json::object const& obj) override {
      NodeBase::Deserialize(obj);
      auto* p_out = RetriveOutput<math::vec4r>(0);
      if (obj.contains("x")) {
        p_out->x() = obj.at("x").as_double();
      }
      if (obj.contains("y")) {
        p_out->y() = obj.at("y").as_double();
      }
      if (obj.contains("z")) {
        p_out->z() = obj.at("z").as_double();
      }
      if (obj.contains("w")) {
        p_out->w() = obj.at("w").as_double();
      }
    }
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

template<idx from, idx to>
class ExpandField : public NodeBase {
  public:
    ExpandField(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<ExpandField<from, to>> fact;
      fact.SetName("Expand_field_" + std::to_string(from) + "r_to_" + std::to_string(to) + "r")
          .SetDescription("Expand a vector field")
          .template AddInput<math::fieldr<from>>("vec", "The vector")
          .template AddOutput<math::fieldr<to>>("out", "The expanded vector");
      for (idx i = from; i < to; i++) {
        fact.template AddInput<real>("dim" + std::to_string(i), "The value of dimension " + std::to_string(i));
      }
      fact.FinalizeAndRegister();
    }

    Status Apply(idx) {
      auto* in_field = RetriveInput<math::fieldr<from>>(0);
      if (in_field == nullptr) {
        return utils::FailedPreconditionError("Input field is not set");
      }
      math::fieldr<to> out_field(to, in_field->cols());
      for (idx i = 0; i < from; i++) {
        out_field.row(i) = in_field->row(i);
      }

      for (idx i = 1; i < to - from; i++) {
        auto* dim = RetriveInput<real>(i);
        if (dim == nullptr) {
          out_field.row(i).setZero();
        } else {
          out_field.row(i).setConstant(*dim);
        }
      }

      SetOutput<math::fieldr<to>>(0, std::move(out_field));
      AX_RETURN_OK();
    }
};

template<idx dim>
class ElementWiseAffineTransform : public NodeBase {
public:
  ElementWiseAffineTransform(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ElementWiseAffineTransform>()
        .SetName("Elemwise_field_aft_" + std::to_string(dim))
        .SetDescription("Elementwise affine transform")
        .template AddInput<math::fieldr<dim>>("in", "Input field")
        .template AddInput<math::vecr<dim>>("scale", "Scale")
        .template AddInput<math::vecr<dim>>("shift", "Shift")
        .template AddOutput<math::fieldr<dim>>("out", "Output field")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* in = RetriveInput<math::fieldr<dim>>(0);
    auto* scale = RetriveInput<math::vecr<dim>>(1);
    auto* shift = RetriveInput<math::vecr<dim>>(2);
    if (in == nullptr) {
      return utils::FailedPreconditionError("Input field is not set");
    }
    math::fieldr<dim> out = *in;
    if (scale != nullptr) {
      for (idx i = 0; i < dim; i++) {
        out.row(i) = out.row(i) * ((*scale)[i]);
      }
    }
    if (shift != nullptr) {
      out.colwise() += *shift;
    }
    SetOutput<math::fieldr<dim>>(0, std::move(out));
    AX_RETURN_OK();
  }
};

template<idx dim>
class MakeConstantVector : public NodeBase {
  public:
    MakeConstantVector(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeConstantVector<dim>>()
          .SetName("Make_constant_vec" + std::to_string(dim) + "r")
          .SetDescription("Make a constant vector")
          .template AddInput<real>("value", "The value of the vector")
          .template AddOutput<math::vecr<dim>>("out", "The constant vector")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      auto* value = RetriveInput<real>(0);
      if (value == nullptr) {
        return utils::FailedPreconditionError("Value is not set");
      }
      SetOutput<math::vecr<dim>>(0, math::vecr<dim>::Constant(*value));
      AX_RETURN_OK();
    }
};

template<idx dim>
class MakeConstantField : public NodeBase {
  public:
    MakeConstantField(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeConstantField<dim>>()
          .SetName("Make_constant_field" + std::to_string(dim) + "r")
          .SetDescription("Make a constant field")
          .template AddInput<real>("value", "The value of the field")
          .template AddInput<idx>("cols", "The number of cols")
          .template AddOutput<math::fieldr<dim>>("out", "The constant field")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      auto* value = RetriveInput<real>(0);
      if (value == nullptr) {
        return utils::FailedPreconditionError("Value is not set");
      }
      auto* cols = RetriveInput<idx>(1);
      if (cols == nullptr) {
        return utils::FailedPreconditionError("Cols is not set");
      }
      SetOutput<math::fieldr<dim>>(0, math::fieldr<dim>::Constant(dim, *cols, *value));
      AX_RETURN_OK();
    }
};

template<idx rows, idx cols>
class MakeConstantMatrix : public NodeBase {
  public:
    MakeConstantMatrix(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeConstantMatrix<rows, cols>>()
          .SetName("Make_constant_mat" + std::to_string(rows)  + std::to_string(cols) + "r")
          .SetDescription("Make a constant matrix")
          .template AddInput<real>("value", "The value of the matrix")
          .template AddOutput<math::matr<rows, cols>>("out", "The constant matrix")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      auto* value = RetriveInput<real>(0);
      if (value == nullptr) {
        return utils::FailedPreconditionError("Value is not set");
      }
      SetOutput<math::matr<rows, cols>>(0, math::matr<rows, cols>::Constant(*value));
      AX_RETURN_OK();
    }
};

class MakeRandom_Real : public NodeBase {
  public:
    MakeRandom_Real(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeRandom_Real>()
          .SetName("Make_random_real")
          .SetDescription("Make a random real number")
          .AddInput<math::vec2r>("range", "The range of the random number")
          .AddOutput<real>("out", "The random real number")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      real low = 0, high = 1;
      if (auto* range = RetriveInput<math::vec2r>(0)) {
        low = range->minCoeff();
        high = range->maxCoeff();
      }
      SetOutput<real>(0, std::uniform_real_distribution<real>(low, high)(generator_));
      AX_RETURN_OK();
    }

    std::default_random_engine generator_;
};

template<idx dim>
class MakeRandom_Vector : public NodeBase {
  public:
    MakeRandom_Vector(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeRandom_Vector<dim>>()
          .SetName("Make_random_vec" + std::to_string(dim) + "r")
          .SetDescription("Make a random vector")
          .template AddInput<math::vec2r>("range", "The range of the random vector")
          .template AddOutput<math::vecr<dim>>("out", "The random vector")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      math::vecr<dim> out;
      real low = 0, high = 1;
      if (auto* range = RetriveInput<math::vec2r>(0)) {
        low = range->minCoeff();
        high = range->maxCoeff();
      }
      for (idx i = 0; i < dim; i++) {
        out[i] = std::uniform_real_distribution<real>(low, high)(generator_);
      }
      SetOutput<math::vecr<dim>>(0, std::move(out));
      AX_RETURN_OK();
    }

    std::default_random_engine generator_;
};

template<idx rows, idx cols>
class MakeRandom_Matrix : public NodeBase {
  public:
    MakeRandom_Matrix(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

    static void register_this() {
      NodeDescriptorFactory<MakeRandom_Matrix<rows, cols>>()
          .SetName("Make_random_mat" + std::to_string(rows) + std::to_string(cols) + "r")
          .SetDescription("Make a random matrix")
          .template AddInput<math::vec2r>("range", "The matrix to convert")
          .template AddOutput<math::matr<rows, cols>>("out", "The random matrix")
          .FinalizeAndRegister();
    }

    Status Apply(idx) {
      math::matr<rows, cols> out;
      real low = 0, high = 1;
      if (auto* range = RetriveInput<math::vec2r>(0)) {
        low = range->minCoeff();
        high = range->maxCoeff();
      }
      for (idx i = 0; i < rows; i++) {
        for (idx j = 0; j < cols; j++) {
          out(i, j) = std::uniform_real_distribution<real>(low, high)(generator_);
        }
      }
      SetOutput<math::matr<rows, cols>>(0, std::move(out));
      AX_RETURN_OK();
    }

    std::default_random_engine generator_;
};

#define DefineConvertFromMatrixXXR(type) \
  class ConvertFromMatrix_##type : public NodeBase { \
  public: \
    ConvertFromMatrix_##type(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {} \
    static void register_this() { \
      NodeDescriptorFactory<ConvertFromMatrix_##type>() \
          .SetName("Convert_matxxr_to_" #type) \
          .SetDescription("Convert a dynamic matrix to a " #type) \
          .AddInput<math::matxxr>("mat", "The matrix to convert") \
          .AddOutput<type>("out", "The converted value") \
          .FinalizeAndRegister(); \
    } \
    Status Apply(idx) override { \
      auto* mat = RetriveInput<math::matxxr>(0); \
      if (mat == nullptr) { \
        return utils::FailedPreconditionError("Input is not set"); \
      } \
      idx rows = type ::RowsAtCompileTime; \
      idx cols = type ::ColsAtCompileTime; \
      if (mat->rows() != rows  && rows != math::dynamic) { \
        return utils::InvalidArgumentError("Rows mismatch"); \
      } else if (mat->cols() != cols && cols != math::dynamic) { \
        return utils::InvalidArgumentError("Cols mismatch"); \
      } \
      SetOutput<type>(0, *mat); \
      AX_RETURN_OK(); \
    } \
  };

#define DefineConvertToMatrixXXR(type) \
  class ConvertToMatrix_##type : public NodeBase { \
  public: \
    ConvertToMatrix_##type(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {} \
    static void register_this() { \
      NodeDescriptorFactory<ConvertToMatrix_##type>() \
          .SetName("Convert_" #type "_to_matxxr") \
          .SetDescription("Convert a " #type " to a dynamic matrix") \
          .AddInput<type>("in", "The value to convert") \
          .AddOutput<math::matxxr>("out", "The converted matrix") \
          .FinalizeAndRegister(); \
    } \
    Status Apply(idx) override { \
      auto* in = RetriveInput<type>(0); \
      if (in == nullptr) { \
        return utils::FailedPreconditionError("Input is not set"); \
      } \
      SetOutput<math::matxxr>(0, math::matxxr(*in)); \
      AX_RETURN_OK(); \
    } \
  };

using namespace ax::math;
DefineConvertFromMatrixXXR(vec2r);
DefineConvertFromMatrixXXR(vec3r);
DefineConvertFromMatrixXXR(vec4r);
DefineConvertFromMatrixXXR(vecxr);
DefineConvertFromMatrixXXR(field1r);
DefineConvertFromMatrixXXR(field2r);
DefineConvertFromMatrixXXR(field3r);
DefineConvertFromMatrixXXR(field4r);

DefineConvertToMatrixXXR(vec2r);
DefineConvertToMatrixXXR(vec3r);
DefineConvertToMatrixXXR(vec4r);
DefineConvertToMatrixXXR(vecxr);
DefineConvertToMatrixXXR(field1r);
DefineConvertToMatrixXXR(field2r);
DefineConvertToMatrixXXR(field3r);
DefineConvertToMatrixXXR(field4r);

void register_math_types_nodes() {
  InputColor3r::register_this();
  InputColor4r::register_this();
  Input_Vec2r::register_this();
  Input_Vec3r::register_this();
  Input_Vec4r::register_this();

  RealVectorGetIndex<2>::register_this();
  RealVectorGetIndex<3>::register_this();
  RealVectorGetIndex<4>::register_this();

  ExpandField<1, 2>::register_this();
  ExpandField<1, 3>::register_this();
  ExpandField<1, 4>::register_this();
  ExpandField<2, 3>::register_this();
  ExpandField<2, 4>::register_this();
  ExpandField<3, 4>::register_this();

  ElementWiseAffineTransform<1>::register_this();
  ElementWiseAffineTransform<2>::register_this();
  ElementWiseAffineTransform<3>::register_this();
  ElementWiseAffineTransform<4>::register_this();

  MakeConstantVector<2>::register_this();
  MakeConstantVector<3>::register_this();
  MakeConstantVector<4>::register_this();

  MakeConstantField<2>::register_this();
  MakeConstantField<3>::register_this();
  MakeConstantField<4>::register_this();

  MakeConstantMatrix<2, 2>::register_this();
  MakeConstantMatrix<3, 3>::register_this();
  MakeConstantMatrix<4, 4>::register_this();

  MakeRandom_Real::register_this();
  MakeRandom_Vector<2>::register_this();
  MakeRandom_Vector<3>::register_this();
  MakeRandom_Vector<4>::register_this();
  MakeRandom_Matrix<2, 2>::register_this();
  MakeRandom_Matrix<3, 3>::register_this();
  MakeRandom_Matrix<4, 4>::register_this();

  ConvertFromMatrix_vec2r::register_this();
  ConvertFromMatrix_vec3r::register_this();
  ConvertFromMatrix_vec4r::register_this();
  ConvertFromMatrix_vecxr::register_this();
  ConvertFromMatrix_field1r::register_this();
  ConvertFromMatrix_field2r::register_this();
  ConvertFromMatrix_field3r::register_this();
  ConvertFromMatrix_field4r::register_this();

  ConvertToMatrix_vec2r::register_this();
  ConvertToMatrix_vec3r::register_this();
  ConvertToMatrix_vec4r::register_this();
  ConvertToMatrix_vecxr::register_this();
  ConvertToMatrix_field1r::register_this();
  ConvertToMatrix_field2r::register_this();
  ConvertToMatrix_field3r::register_this();
  ConvertToMatrix_field4r::register_this();
}

}  // namespace ax::nodes
