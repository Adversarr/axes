//
// Created by adversarr on 4/13/24.
//
#include "ax/nodes/stl_types.hpp"

#include <imgui_node_editor.h>

#include "ax/core/entt.hpp"
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
    using T = class_name;                                                                  \
    Input_##class_name(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {} \
    static void register_this() {                                                          \
      NodeDescriptorFactory<Input_##class_name>{}                                          \
          .SetName("Input_" #class_name)                                                   \
          .SetDescription("User Input for " #class_name)                                   \
          .AddOutput<class_name>("out", "value from input")                              \
          .FinalizeAndRegister();                                                          \
      add_custom_node_render(typeid(Input_##class_name), CustomNodeRender{[](NodeBase* n) {       \
              auto node = dynamic_cast<Input_##class_name *>(n); begin_draw_node(n); draw_node_header_default(n);
#define DefineInputNodeEnd(method)                     \
  end_draw_node();                                     \
  }                                                    \
  });                                                  \
  }                                                    \
  boost::json::object Serialize() const {              \
    boost::json::object obj;                           \
    obj["value"] = *RetriveOutput<T>(0);               \
    return obj;                                        \
  }                                                    \
  void Deserialize(boost::json::object const& obj) {   \
    if (obj.contains("value")) {                       \
      *RetriveOutput<T>(0) = obj.at("value").method(); \
    }                                                  \
  }                                                    \
  }

DefineInputNodeBegin(int) auto* p_put = node->RetriveOutput<int>(0);
ImGui::SetNextItemWidth(100);
ImGui::InputInt("value", p_put);
ImGui::SameLine();
ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
ed::EndPin();
DefineInputNodeEnd(as_int64);

DefineInputNodeBegin(real) auto* p_put = node->RetriveOutput<real>(0);
ImGui::SetNextItemWidth(100);
ImGui::InputDouble("value", p_put);
ImGui::SameLine();
ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
ed::EndPin();
DefineInputNodeEnd(as_double);

DefineInputNodeBegin(idx) auto* p_put = node->RetriveOutput<idx>(0);
ImGui::SetNextItemWidth(100);
ImGui::InputScalar("value", ImGuiDataType_S64, p_put);
ImGui::SameLine();
ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
ed::EndPin();
DefineInputNodeEnd(as_int64);

class Input_bool : public NodeBase {
public:
  Input_bool(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  static void register_this() {
    NodeDescriptorFactory<Input_bool>{}
        .SetName("Input_bool")
        .SetDescription("User Input for bool")
        .AddOutput<bool>("out", "value from input")
        .FinalizeAndRegister();
    add_custom_node_render(typeid(Input_bool), CustomNodeRender{[](NodeBase* n) {
                             begin_draw_node(n);
                             draw_node_header_default(n);
                             auto node = dynamic_cast<Input_bool*>(n);
                             auto* p_put = node->RetriveOutput<bool>(0);
                             ImGui::SetNextItemWidth(100);
                             ImGui::Checkbox("value", p_put);
                             ImGui::SameLine();
                             ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
                             ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
                             ed::EndPin();
                             end_draw_node();
                           }});
  }

  Status OnConstruct() override {
    *RetriveOutput<bool>(0) = false;
    AX_RETURN_OK();
  }

  boost::json::object Serialize() const override {
    boost::json::object obj;
    obj["value"] = *RetriveOutput<bool>(0);
    return obj;
  }

  void Deserialize(boost::json::object const& obj) override {
    if (obj.contains("value")) {
      *RetriveOutput<bool>(0) = obj.at("value").as_bool();
    }
  }
};

class Input_string : public NodeBase {
public:
  Input_string(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  static void register_this() {
    NodeDescriptorFactory<Input_string>{}
        .SetName("Input_string")
        .SetDescription("User Input for string")
        .AddOutput<string>("out", "value from input")
        .FinalizeAndRegister();
    add_custom_node_render(
        typeid(Input_string), CustomNodeRender{[](NodeBase* n) {
          begin_draw_node(n);
          draw_node_header_default(n);
          auto node = dynamic_cast<Input_string*>(n);
          auto* p_put = node->RetriveOutput<std::string>(0);
          ImGui::SetNextItemWidth(100);
          if (ImGui::InputText("value", node->buffer, 256, ImGuiInputTextFlags_EnterReturnsTrue)) {
            *p_put = node->buffer;
          }
          ImGui::SameLine();
          ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
          end_draw_node();
        }});
  }

  boost::json::object Serialize() const override {
    boost::json::object obj;
    obj["value"] = std::string(buffer);
    return obj;
  }

  void Deserialize(boost::json::object const& obj) override {
    if (obj.contains("value")) {
      auto const& s = (*RetriveOutput<string>(0) = obj.at("value").as_string().c_str());
      memcpy(buffer, s.c_str(), s.size());
    }
  }

private:
  char buffer[256]{0};
};
#define DefineConversionNodeBegin(from_type, to_type)                          \
  class Convert_##from_type##_to_##to_type : public NodeBase {                 \
  public:                                                                      \
    Convert_##from_type##_to_##to_type(NodeDescriptor const* descript, idx id) \
        : NodeBase(descript, id) {}                                            \
    static void register_this() {                                              \
      NodeDescriptorFactory<Convert_##from_type##_to_##to_type>{}              \
          .SetName("Convert_" #from_type "_to_" #to_type)                      \
          .SetDescription("Convert " #from_type " to " #to_type)               \
          .AddInput<from_type>("in", "input value")                         \
          .AddOutput<to_type>("out", "output value")                        \
          .FinalizeAndRegister();                                              \
    }                                                                          \
    Status Apply(idx) override {                                               \
      auto* p_get = RetriveInput<from_type>(0);                                \
      if (!p_get) {                                                            \
        return utils::InvalidArgumentError("input is null");                   \
      }                                                                        \
      auto* p_put = RetriveOutput<to_type>(0);                                 \
      *p_put = static_cast<to_type>(*p_get);                                   \
      AX_RETURN_OK();                                                          \
    }                                                                          \
  }

DefineConversionNodeBegin(int, idx);
DefineConversionNodeBegin(idx, int);
DefineConversionNodeBegin(int, real);
DefineConversionNodeBegin(real, int);
DefineConversionNodeBegin(real, idx);
DefineConversionNodeBegin(idx, real);

DefineConversionNodeBegin(bool, int);
DefineConversionNodeBegin(int, bool);
DefineConversionNodeBegin(bool, real);
DefineConversionNodeBegin(real, bool);
DefineConversionNodeBegin(bool, idx);
DefineConversionNodeBegin(idx, bool);

#define DefineConvertionToString(from_type)                                 \
  class Convert_##from_type##_to_string : public NodeBase {                 \
  public:                                                                   \
    Convert_##from_type##_to_string(NodeDescriptor const* descript, idx id) \
        : NodeBase(descript, id) {}                                         \
    static void register_this() {                                           \
      NodeDescriptorFactory<Convert_##from_type##_to_string>{}              \
          .SetName("Convert_" #from_type "_to_string")                      \
          .AddInput<from_type>("in", "input value")                      \
          .AddOutput<string>("out", "output value")                      \
          .FinalizeAndRegister();                                           \
    }                                                                       \
    Status Apply(idx) override {                                            \
      auto* p_get = RetriveInput<from_type>(0);                             \
      if (!p_get) {                                                         \
        return utils::InvalidArgumentError("input is null");                \
      }                                                                     \
      auto* p_put = RetriveOutput<string>(0);                               \
      *p_put = std::to_string(*p_get);                                      \
      AX_RETURN_OK();                                                       \
    }                                                                       \
  }

#define DefineConvertionFromString(to_type, method)                     \
  class Convert_string_to_##to_type : public NodeBase {                 \
  public:                                                               \
    Convert_string_to_##to_type(NodeDescriptor const* descript, idx id) \
        : NodeBase(descript, id) {}                                     \
    static void register_this() {                                       \
      NodeDescriptorFactory<Convert_string_to_##to_type>{}              \
          .SetName("Convert_string_to_" #to_type)                       \
          .SetDescription("Convert string to " #to_type)                \
          .AddInput<string>("in", "input value")                     \
          .AddOutput<to_type>("out", "output value")                 \
          .FinalizeAndRegister();                                       \
    }                                                                   \
    Status Apply(idx) override {                                        \
      auto* p_get = RetriveInput<string>(0);                            \
      if (!p_get) {                                                     \
        return utils::InvalidArgumentError("input is null");            \
      }                                                                 \
      auto* p_put = RetriveOutput<to_type>(0);                          \
      *p_put = method(*p_get);                                          \
      AX_RETURN_OK();                                                   \
    }                                                                   \
  }

DefineConvertionToString(int);
DefineConvertionToString(real);
DefineConvertionToString(idx);
DefineConvertionFromString(int, std::stoi);
DefineConvertionFromString(real, std::stod);
DefineConvertionFromString(idx, std::stoll);

bool from_string_to_bool(string const& str) { return str == "true"; }

bool from_int_to_bool(int const& i) { return i != 0; }

int from_bool_to_int(bool const& b) { return b ? 1 : 0; }

string from_bool_to_string(bool const& b) { return b ? "true" : "false"; }

DefineConvertionFromString(bool, from_string_to_bool);

class Convert_bool_to_string : public NodeBase {
public:
  Convert_bool_to_string(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  static void register_this() {
    NodeDescriptorFactory<Convert_bool_to_string>{}
        .SetName(
            "Convert_"
            "bool"
            "_to_string")
        .AddInput<bool>("in", "input value")
        .AddOutput<string>("out", "output value")
        .FinalizeAndRegister();
  }
  Status Apply(idx) override {
    auto* p_get = RetriveInput<bool>(0);
    if (!p_get) {
      return utils ::InvalidArgumentError("input is null");
    }
    auto* p_put = RetriveOutput<string>(0);
    *p_put = from_bool_to_string(*p_get);
    AX_RETURN_OK();
  }
};

#define DefineLogtoConsoleNode(type, severity)                                 \
  class Log_##type##_to_console_##severity : public NodeBase {                 \
  public:                                                                      \
    Log_##type##_to_console_##severity(NodeDescriptor const* descript, idx id) \
        : NodeBase(descript, id) {}                                            \
    static void register_this() {                                              \
      NodeDescriptorFactory<Log_##type##_to_console_##severity>{}              \
          .SetName("Log_" #type "_" #severity)                                 \
          .SetDescription("Log " #type " to console (" #severity ")")          \
          .AddInput<type>("in", "input value")                              \
          .FinalizeAndRegister();                                              \
    }                                                                          \
    Status Apply(idx f) override {                                             \
      auto* p_get = RetriveInput<type>(0);                                     \
      if (!p_get) {                                                            \
        return utils::InvalidArgumentError("input is null");                   \
      }                                                                        \
      AX_LOG(severity) << "Frame ID: " << f << #type " value: " << *p_get;     \
      AX_RETURN_OK();                                                          \
    }                                                                          \
  }

DefineLogtoConsoleNode(int, INFO);
DefineLogtoConsoleNode(real, INFO);
DefineLogtoConsoleNode(idx, INFO);
DefineLogtoConsoleNode(string, INFO);
DefineLogtoConsoleNode(bool, INFO);
DefineLogtoConsoleNode(int, WARNING);
DefineLogtoConsoleNode(real, WARNING);
DefineLogtoConsoleNode(idx, WARNING);
DefineLogtoConsoleNode(string, WARNING);
DefineLogtoConsoleNode(bool, WARNING);
DefineLogtoConsoleNode(int, ERROR);
DefineLogtoConsoleNode(real, ERROR);
DefineLogtoConsoleNode(idx, ERROR);
DefineLogtoConsoleNode(string, ERROR);
DefineLogtoConsoleNode(bool, ERROR);

#define DefineOperatorForType(type, op, name)                                                    \
  class Operator_##type##_##name : public NodeBase {                                             \
  public:                                                                                        \
    Operator_##type##_##name(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {} \
    static void register_this() {                                                                \
      NodeDescriptorFactory<Operator_##type##_##name>{}                                          \
          .SetName(#type "_" #op)                                                                \
          .SetDescription("Operation " #op " for " #type)                                        \
          .AddInput<type>("in1", "input value 1")                                             \
          .AddInput<type>("in2", "input value 2")                                             \
          .AddOutput<type>("out", "output value")                                             \
          .FinalizeAndRegister();                                                                \
    }                                                                                            \
    Status Apply(idx) override {                                                                 \
      auto* p_get1 = RetriveInput<type>(0);                                                      \
      auto* p_get2 = RetriveInput<type>(1);                                                      \
      if (!p_get1 || !p_get2) {                                                                  \
        return utils::InvalidArgumentError("input is null");                                     \
      }                                                                                          \
      auto* p_put = RetriveOutput<type>(0);                                                      \
      *p_put = *p_get1 op * p_get2;                                                              \
      AX_RETURN_OK();                                                                            \
    }                                                                                            \
  }

#define DefineAllOperatorsForType(type) \
  DefineOperatorForType(type, +, add);  \
  DefineOperatorForType(type, -, sub);  \
  DefineOperatorForType(type, *, mul);  \
  DefineOperatorForType(type, /, div);

DefineAllOperatorsForType(int);
DefineAllOperatorsForType(real);
DefineAllOperatorsForType(idx);

class CreateEntity : public NodeBase {
public:
  CreateEntity(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  static void register_this() {
    NodeDescriptorFactory<CreateEntity>{}
        .SetName("Create_entity")
        .SetDescription("Create an entity.")
        .AddOutput<Entity>("entity", "The entity.")
        .FinalizeAndRegister();

    add_custom_node_render<CreateEntity>([](NodeBase* n) {
      begin_draw_node(n);
      draw_node_header_default(n);
      auto node = dynamic_cast<CreateEntity*>(n);
      auto ent = node->ent_;

      ImGui::Text("Entity: %d", entt::to_integral(ent));
      ImGui::SameLine();
      ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
      ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
      ed::EndPin();
      end_draw_node();
    });
  }

  Status OnConstruct() override {
    ent_ = create_entity();
    *RetriveOutput<Entity>(0) = ent_;
    AX_RETURN_OK();
  }

  void OnDestroy() override {
    if (ent_ != entt::null) {
      destroy_entity(ent_);
    }
    *RetriveOutput<Entity>(0) = ent_;
  }

private:
  Entity ent_ = entt::null;
};

class GetFrameId : public NodeBase {
public:
  GetFrameId(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  static void register_this() {
    NodeDescriptorFactory<GetFrameId>{}
        .SetName("Get_frame_id")
        .SetDescription("Get the frame id.")
        .AddOutput<idx>("frame_id", "The frame id.")
        .FinalizeAndRegister();
  }

  Status Apply(idx frame_id) {
    SetOutput<idx>(0, frame_id);
    AX_RETURN_OK();
  }
};

class StringConcat : public NodeBase {
  public:
  StringConcat(NodeDescriptor const* descript, idx id) : NodeBase(descript, id) {}
  static void register_this() {
    NodeDescriptorFactory<StringConcat>{}
        .SetName("String_concat")
        .SetDescription("Concatenate two strings.")
        .AddInput<string>("in1", "input value 1")
        .AddInput<string>("in2", "input value 2")
        .AddOutput<string>("out", "output value")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto* p_get1 = RetriveInput<string>(0);
    auto* p_get2 = RetriveInput<string>(1);
    if (!p_get1 || !p_get2) {
      return utils::InvalidArgumentError("input is null");
    }
    auto* p_put = RetriveOutput<string>(0);
    *p_put = *p_get1 + *p_get2;
    AX_RETURN_OK();
  }
};



void register_stl_types() {
  Input_int::register_this();
  Input_real::register_this();
  Input_idx::register_this();
  Input_string::register_this();
  Input_bool::register_this();

  Convert_int_to_idx::register_this();
  Convert_idx_to_int::register_this();
  Convert_int_to_real::register_this();
  Convert_real_to_int::register_this();
  Convert_real_to_idx::register_this();
  Convert_idx_to_real::register_this();

  Convert_int_to_string::register_this();
  Convert_real_to_string::register_this();
  Convert_idx_to_string::register_this();
  Convert_string_to_int::register_this();
  Convert_string_to_real::register_this();
  Convert_string_to_idx::register_this();

  Convert_bool_to_int::register_this();
  Convert_int_to_bool::register_this();
  Convert_bool_to_real::register_this();
  Convert_real_to_bool::register_this();
  Convert_bool_to_idx::register_this();
  Convert_idx_to_bool::register_this();

  Convert_bool_to_string::register_this();
  Convert_string_to_bool::register_this();

  Log_int_to_console_INFO::register_this();
  Log_real_to_console_INFO::register_this();
  Log_idx_to_console_INFO::register_this();
  Log_string_to_console_INFO::register_this();
  Log_bool_to_console_INFO::register_this();
  Log_int_to_console_WARNING::register_this();
  Log_real_to_console_WARNING::register_this();
  Log_idx_to_console_WARNING::register_this();
  Log_string_to_console_WARNING::register_this();
  Log_bool_to_console_WARNING::register_this();
  Log_int_to_console_ERROR::register_this();
  Log_real_to_console_ERROR::register_this();
  Log_idx_to_console_ERROR::register_this();
  Log_string_to_console_ERROR::register_this();
  Log_bool_to_console_ERROR::register_this();

  Operator_int_add::register_this();
  Operator_int_sub::register_this();
  Operator_int_mul::register_this();
  Operator_int_div::register_this();
  Operator_real_add::register_this();
  Operator_real_sub::register_this();
  Operator_real_mul::register_this();
  Operator_real_div::register_this();
  Operator_idx_add::register_this();
  Operator_idx_sub::register_this();
  Operator_idx_mul::register_this();
  Operator_idx_div::register_this();

  CreateEntity::register_this();
  GetFrameId::register_this();
  StringConcat::register_this();
}
}  // namespace ax::nodes
