// #include "ax/nodes/math_types.hpp"
//
// #include <imgui.h>
// #include <imgui_node_editor.h>
//
// #include <random>
//
// #include "ax/core/excepts.hpp"
// #include "ax/graph/node.hpp"
// #include "ax/graph/render.hpp"
// #include "ax/math/common.hpp"
// #include "ax/math/functional.hpp"
//
// namespace ed = ax::NodeEditor;
//
// namespace ax::nodes {
// using namespace graph;
//
// const char names[4][2] = {"x", "y", "z", "w"};
//
// class InputColor3r : public NodeBase {
// public:
//   InputColor3r(NodeDescriptor const* descriptor, size_t id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<InputColor3r>()
//         .SetName("Input_color3r")
//         .SetDescription("Input a color")
//         .AddOutput<math::RealVector3>("out", "The color")
//         .FinalizeAndRegister();
//
//     add_custom_node_render(typeid(InputColor3r), {[](NodeBase* node) {
//                              begin_draw_node(node);
//                              draw_node_header_default(node);
//                              auto n = static_cast<InputColor3r*>(node);
//                              ImVec4 color(n->color_.x(), n->color_.y(), n->color_.z(), 1.0f);
//                              ImGui::SetNextItemWidth(100);
//                              if (ImGui::ColorButton("Color Picker", color)) {
//                                ImGui::OpenPopup("ColorPicker");
//                              }
//                              ImGui::SameLine();
//                              ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
//                              ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//                              ed::EndPin();
//                              end_draw_node();
//
//                              ed::Suspend();
//                              ImGui::PushID(node);
//                              if (ImGui::BeginPopup("ColorPicker")) {
//                                ImGui::ColorPicker3("Color", n->color_.data(),
//                                                    ImGuiColorEditFlags_Float);
//                                ImGui::EndPopup();
//                              }
//                              ImGui::PopID();
//                              ed::Resume();
//                            }});
//   }
//
//   void PreApply() final { SetOutput<math::RealVector3>(0, color_.cast<real>()); }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     obj["color"] = boost::json::array{color_.x(), color_.y(), color_.z()};
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     NodeBase::Deserialize(obj);
//     if (!obj.contains("color")) {
//       return;
//     }
//     auto arr = obj.at("color").as_array();
//     color_ = math::FloatVector3(arr[0].as_double(), arr[1].as_double(), arr[2].as_double());
//   }
//
//   math::FloatVector3 color_;
// };
//
// class InputColor4r : public NodeBase {
// public:
//   InputColor4r(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<InputColor4r>()
//         .SetName("Input_color4r")
//         .SetDescription("Input a color")
//         .AddOutput<math::RealVector4>("out", "The color")
//         .FinalizeAndRegister();
//
//     add_custom_node_render(
//         typeid(InputColor4r), {[](NodeBase* node) {
//           begin_draw_node(node);
//           draw_node_header_default(node);
//           auto n = static_cast<InputColor4r*>(node);
//           ImVec4 color(n->color_.x(), n->color_.y(), n->color_.z(), n->color_.w());
//           ImGui::SetNextItemWidth(100);
//           if (ImGui::ColorButton("Color Picker", color)) {
//             ImGui::OpenPopup("ColorPicker");
//           }
//
//           ImGui::SameLine();
//           ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
//           ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//           ed::EndPin();
//           end_draw_node();
//
//           ed::Suspend();
//           ImGui::PushID(node);
//           if (ImGui::BeginPopup("ColorPicker")) {
//             ImGui::ColorPicker4("Color", n->color_.data(), ImGuiColorEditFlags_Float);
//             ImGui::EndPopup();
//           }
//           ImGui::PopID();
//           ed::Resume();
//         }});
//   }
//
//   void Apply(size_t) final { SetOutput<math::RealVector4>(0, color_.cast<real>()); }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     obj["color"] = boost::json::array{color_.x(), color_.y(), color_.z(), color_.w()};
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     NodeBase::Deserialize(obj);
//     if (!obj.contains("color")) {
//       return;
//     }
//     auto arr = obj.at("color").as_array();
//     color_ = math::FloatVec4(arr[0].as_double(), arr[1].as_double(), arr[2].as_double(),
//                          arr[3].as_double());
//   }
//
//   math::FloatVec4 color_;
// };
//
// class Input_Vec2r : public NodeBase {
// public:
//   Input_Vec2r(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<Input_Vec2r>()
//         .SetName("Input_RealVector2")
//         .SetDescription("Input a 2D vector")
//         .AddOutput<math::RealVector2>("out", "The vector")
//         .FinalizeAndRegister();
//
//     add_custom_node_render(typeid(Input_Vec2r), {[](NodeBase* node) {
//                              begin_draw_node(node);
//                              draw_node_header_default(node);
//                              auto n = static_cast<Input_Vec2r*>(node);
//                              auto* p_out = n->RetriveOutput<math::RealVector2>(0);
//                              for (int i = 0; i < 2; i++) {
//                                ImGui::SetNextItemWidth(100);
//                                ImGui::InputDouble(names[i], p_out->data() + i);
//                              }
//                              ImGui::SameLine();
//                              ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
//                              ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//                              ed::EndPin();
//                              end_draw_node();
//                            }});
//   }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     auto* p_out = RetriveOutput<math::RealVector2>(0);
//     obj["x"] = p_out->x();
//     obj["y"] = p_out->y();
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     NodeBase::Deserialize(obj);
//     auto* p_out = RetriveOutput<math::RealVector2>(0);
//     if (obj.contains("x")) {
//       p_out->x() = obj.at("x").as_double();
//     }
//     if (obj.contains("y")) {
//       p_out->y() = obj.at("y").as_double();
//     }
//   }
// };
//
// class Input_Vec3r : public NodeBase {
// public:
//   Input_Vec3r(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<Input_Vec3r>()
//         .SetName("Input_RealVector3")
//         .SetDescription("Input a 3D vector")
//         .AddOutput<math::RealVector3>("out", "The vector")
//         .FinalizeAndRegister();
//
//     add_custom_node_render(typeid(Input_Vec3r), {[](NodeBase* node) {
//                              begin_draw_node(node);
//                              draw_node_header_default(node);
//                              auto n = static_cast<Input_Vec3r*>(node);
//                              auto* p_out = n->RetriveOutput<math::RealVector3>(0);
//                              for (int i = 0; i < 3; i++) {
//                                ImGui::SetNextItemWidth(100);
//                                ImGui::InputDouble(names[i], p_out->data() + i);
//                              }
//                              ImGui::SameLine();
//                              ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
//                              ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//                              ed::EndPin();
//                              end_draw_node();
//                            }});
//   }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     auto* p_out = RetriveOutput<math::RealVector3>(0);
//     obj["x"] = p_out->x();
//     obj["y"] = p_out->y();
//     obj["z"] = p_out->z();
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     NodeBase::Deserialize(obj);
//     auto* p_out = RetriveOutput<math::RealVector3>(0);
//     if (obj.contains("x")) {
//       p_out->x() = obj.at("x").as_double();
//     }
//     if (obj.contains("y")) {
//       p_out->y() = obj.at("y").as_double();
//     }
//     if (obj.contains("z")) {
//       p_out->z() = obj.at("z").as_double();
//     }
//   }
// };
//
// class Input_Vec4r : public NodeBase {
// public:
//   Input_Vec4r(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<Input_Vec4r>()
//         .SetName("Input_RealVector4")
//         .SetDescription("Input a 4D vector")
//         .AddOutput<math::RealVector4>("out", "The vector")
//         .FinalizeAndRegister();
//
//     add_custom_node_render(typeid(Input_Vec4r), {[](NodeBase* node) {
//                              begin_draw_node(node);
//                              draw_node_header_default(node);
//                              auto n = static_cast<Input_Vec4r*>(node);
//                              auto* p_out = n->RetriveOutput<math::RealVector4>(0);
//                              for (int i = 0; i < 4; i++) {
//                                ImGui::SetNextItemWidth(100);
//                                ImGui::InputDouble(names[i], p_out->data() + i);
//                              }
//                              ImGui::SameLine();
//                              ed::BeginPin(n->GetOutput(0)->id_, ed::PinKind::Output);
//                              ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//                              ed::EndPin();
//                              end_draw_node();
//                            }});
//   }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     auto* p_out = RetriveOutput<math::RealVector4>(0);
//     obj["x"] = p_out->x();
//     obj["y"] = p_out->y();
//     obj["z"] = p_out->z();
//     obj["w"] = p_out->w();
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     NodeBase::Deserialize(obj);
//     auto* p_out = RetriveOutput<math::RealVector4>(0);
//     if (obj.contains("x")) {
//       p_out->x() = obj.at("x").as_double();
//     }
//     if (obj.contains("y")) {
//       p_out->y() = obj.at("y").as_double();
//     }
//     if (obj.contains("z")) {
//       p_out->z() = obj.at("z").as_double();
//     }
//     if (obj.contains("w")) {
//       p_out->w() = obj.at("w").as_double();
//     }
//   }
// };
//
// template <Index dim> class RealVectorGetIndex : public NodeBase {
// public:
//   RealVectorGetIndex(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<RealVectorGetIndex<dim>>()
//         .SetName("Vec" + std::to_string(dim) + "r_get_index")
//         .SetDescription("Get a component of a vector")
//         .template AddInput<math::RealVector<dim>>("vec", "The vector")
//         .template AddInput<Index>("index", "The index of the component to get")
//         .template AddOutput<real>("out", "The component")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     auto* vector = RetriveInput<math::RealVector<dim>>(0);
//     auto* index = RetriveInput<Index>(1);
//     if (vector == nullptr) {
//       throw make_invalid_argument("Vector is not set");
//     }
//     if (index == nullptr) {
//       throw make_invalid_argument("Index is not set");
//     }
//     if (*index < 0 || *index >= dim) {
//       throw make_invalid_argument("Index out of bounds");
//     }
//     SetOutput<real>(0, (*vector)[*index]);
//   }
// };
//
// template <Index from, Index to> class ExpandField : public NodeBase {
// public:
//   ExpandField(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<ExpandField<from, to>> fact;
//     fact.SetName("Expand_field_" + std::to_string(from) + "r_to_" + std::to_string(to) + "r")
//         .SetDescription("Expand a vector field")
//         .template AddInput<math::RealField<from>>("vec", "The vector")
//         .template AddOutput<math::RealField<to>>("out", "The expanded vector");
//     for (Index i = from; i < to; i++) {
//       fact.template AddInput<real>("dim" + std::to_string(i),
//                                    "The value of dimension " + std::to_string(i));
//     }
//     fact.FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     auto* in_field = RetriveInput<math::RealField<from>>(0);
//     if (in_field == nullptr) {
//       throw make_invalid_argument("Input field is not set");
//     }
//     math::RealField<to> out_field(to, in_field->cols());
//     for (Index i = 0; i < from; i++) {
//       out_field.row(i) = in_field->row(i);
//     }
//
//     for (Index i = 1; i < to - from; i++) {
//       auto* dim = RetriveInput<real>(i);
//       if (dim == nullptr) {
//         out_field.row(i).setZero();
//       } else {
//         out_field.row(i).setConstant(*dim);
//       }
//     }
//
//     SetOutput<math::RealField<to>>(0, std::move(out_field));
//   }
// };
//
// template <Index dim> class ElementWiseAffineTransform : public NodeBase {
// public:
//   ElementWiseAffineTransform(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<ElementWiseAffineTransform>()
//         .SetName("Elemwise_field_aft_" + std::to_string(dim))
//         .SetDescription("Elementwise affine transform")
//         .template AddInput<math::RealField<dim>>("field", "Input field")
//         .template AddInput<math::RealVector<dim>>("scale", "Scale")
//         .template AddInput<math::RealVector<dim>>("shift", "Shift")
//         .template AddOutput<math::RealField<dim>>("out", "Output field")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) override {
//     auto* in = RetriveInput<math::RealField<dim>>(0);
//     auto* scale = RetriveInput<math::RealVector<dim>>(1);
//     auto* shift = RetriveInput<math::RealVector<dim>>(2);
//     if (in == nullptr) {
//       throw make_invalid_argument("Input field is not set");
//     }
//     math::RealField<dim> out = *in;
//     if (scale != nullptr) {
//       for (Index i = 0; i < dim; i++) {
//         out.row(i) = out.row(i) * ((*scale)[i]);
//       }
//     }
//     if (shift != nullptr) {
//       out.colwise() += *shift;
//     }
//     SetOutput<math::RealField<dim>>(0, std::move(out));
//   }
// };
//
// template <Index dim> class MakeConstantVector : public NodeBase {
// public:
//   MakeConstantVector(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<MakeConstantVector<dim>>()
//         .SetName("Make_constant_vec" + std::to_string(dim) + "r")
//         .SetDescription("Make a constant vector")
//         .template AddInput<real>("value", "The value of the vector")
//         .template AddOutput<math::RealVector<dim>>("out", "The constant vector")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     auto* value = RetriveInput<real>(0);
//     if (value == nullptr) {
//       throw make_invalid_argument("Value is not set");
//     }
//     SetOutput<math::RealVector<dim>>(0, math::RealVector<dim>::Constant(*value));
//   }
// };
//
// template <Index dim> class MakeConstantField : public NodeBase {
// public:
//   MakeConstantField(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<MakeConstantField<dim>>()
//         .SetName("Make_constant_field" + std::to_string(dim) + "r")
//         .SetDescription("Make a constant field")
//         .template AddInput<real>("value", "The value of the field")
//         .template AddInput<Index>("cols", "The number of cols")
//         .template AddOutput<math::RealField<dim>>("out", "The constant field")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     auto* value = RetriveInput<real>(0);
//     if (value == nullptr) {
//       throw make_invalid_argument("Value is not set");
//     }
//     auto* cols = RetriveInput<Index>(1);
//     if (cols == nullptr) {
//       throw make_invalid_argument("Cols is not set");
//     }
//     SetOutput<math::RealField<dim>>(0, math::RealField<dim>::Constant(dim, *cols, *value));
//   }
// };
//
// template <Index rows, Index cols> class MakeConstantMatrix : public NodeBase {
// public:
//   MakeConstantMatrix(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<MakeConstantMatrix<rows, cols>>()
//         .SetName("Make_constant_mat" + std::to_string(rows) + std::to_string(cols) + "r")
//         .SetDescription("Make a constant matrix")
//         .template AddInput<real>("value", "The value of the matrix")
//         .template AddOutput<math::RealMatrix<rows, cols>>("out", "The constant matrix")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     auto* value = RetriveInput<real>(0);
//     if (value == nullptr) {
//       throw make_invalid_argument("Value is not set");
//     }
//     SetOutput<math::RealMatrix<rows, cols>>(0, math::RealMatrix<rows, cols>::Constant(*value));
//   }
// };
//
// class MakeRandom_Real : public NodeBase {
// public:
//   MakeRandom_Real(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<MakeRandom_Real>()
//         .SetName("Make_random_real")
//         .SetDescription("Make a random real number")
//         .AddInput<math::RealVector2>("range", "The range of the random number")
//         .AddOutput<real>("out", "The random real number")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     real low = 0, high = 1;
//     if (auto* range = RetriveInput<math::RealVector2>(0)) {
//       low = range->minCoeff();
//       high = range->maxCoeff();
//     }
//     SetOutput<real>(0, std::uniform_real_distribution<real>(low, high)(generator_));
//   }
//
//   std::default_random_engine generator_;
// };
//
// template <Index dim> class MakeRandom_Vector : public NodeBase {
// public:
//   MakeRandom_Vector(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<MakeRandom_Vector<dim>>()
//         .SetName("Make_random_vec" + std::to_string(dim) + "r")
//         .SetDescription("Make a random vector")
//         .template AddInput<math::RealVector2>("range", "The range of the random vector")
//         .template AddOutput<math::RealVector<dim>>("out", "The random vector")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     math::RealVector<dim> out;
//     real low = 0, high = 1;
//     if (auto* range = RetriveInput<math::RealVector2>(0)) {
//       low = range->minCoeff();
//       high = range->maxCoeff();
//     }
//     for (Index i = 0; i < dim; i++) {
//       out[i] = std::uniform_real_distribution<real>(low, high)(generator_);
//     }
//     SetOutput<math::RealVector<dim>>(0, std::move(out));
//   }
//
//   std::default_random_engine generator_;
// };
//
// template <Index rows, Index cols> class MakeRandom_Matrix : public NodeBase {
// public:
//   MakeRandom_Matrix(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<MakeRandom_Matrix<rows, cols>>()
//         .SetName("Make_random_mat" + std::to_string(rows) + std::to_string(cols) + "r")
//         .SetDescription("Make a random matrix")
//         .template AddInput<math::RealVector2>("range", "The matrix to convert")
//         .template AddOutput<math::RealMatrix<rows, cols>>("out", "The random matrix")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) {
//     math::RealMatrix<rows, cols> out;
//     real low = 0, high = 1;
//     if (auto* range = RetriveInput<math::RealVector2>(0)) {
//       low = range->minCoeff();
//       high = range->maxCoeff();
//     }
//     for (Index i = 0; i < rows; i++) {
//       for (Index j = 0; j < cols; j++) {
//         out(i, j) = std::uniform_real_distribution<real>(low, high)(generator_);
//       }
//     }
//     SetOutput<math::RealMatrix<rows, cols>>(0, std::move(out));
//   }
//
//   std::default_random_engine generator_;
// };
//
// #define DefineConvertFromMatrixXXR(type)                               \
//   class ConvertFromMatrix_##type : public NodeBase {                   \
//   public:                                                              \
//     ConvertFromMatrix_##type(NodeDescriptor const* descriptor, Index id) \
//         : NodeBase(descriptor, id) {}                                  \
//     static void register_this() {                                      \
//       NodeDescriptorFactory<ConvertFromMatrix_##type>()                \
//           .SetName("Convert_matxxr_to_" #type)                         \
//           .SetDescription("Convert a dynamic matrix to a " #type)      \
//           .AddInput<math::matxxr>("mat", "The matrix to convert")      \
//           .AddOutput<type>("out", "The converted value")               \
//           .FinalizeAndRegister();                                      \
//     }                                                                  \
//     void Apply(size_t) override {                                      \
//       auto* mat = RetriveInput<math::matxxr>(0);                       \
//       if (mat == nullptr) {                                            \
//         throw make_invalid_argument("Input is not set");               \
//       }                                                                \
//       Index rows = type ::RowsAtCompileTime;                             \
//       Index cols = type ::ColsAtCompileTime;                             \
//       if (mat->rows() != rows && rows != math::dynamic) {              \
//         throw make_invalid_argument("Rows mismatch");                  \
//       } else if (mat->cols() != cols && cols != math::dynamic) {       \
//         throw make_invalid_argument("Cols mismatch");                  \
//       }                                                                \
//       SetOutput<type>(0, *mat);                                        \
//     }                                                                  \
//   };
//
// #define DefineConvertToMatrixXXR(type)                                                             \
//   class ConvertToMatrix_##type : public NodeBase {                                                 \
//   public:                                                                                          \
//     ConvertToMatrix_##type(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {} \
//     static void register_this() {                                                                  \
//       NodeDescriptorFactory<ConvertToMatrix_##type>()                                              \
//           .SetName("Convert_" #type "_to_matxxr")                                                  \
//           .SetDescription("Convert a " #type " to a dynamic matrix")                               \
//           .AddInput<type>("in", "The value to convert")                                            \
//           .AddOutput<math::matxxr>("out", "The converted matrix")                                  \
//           .FinalizeAndRegister();                                                                  \
//     }                                                                                              \
//     void Apply(size_t) override {                                                                  \
//       auto* in = RetriveInput<type>(0);                                                            \
//       if (in == nullptr) {                                                                         \
//         throw make_invalid_argument("Input is not set");                                           \
//       }                                                                                            \
//       SetOutput<math::matxxr>(0, math::matxxr(*in));                                               \
//     }                                                                                              \
//   };
//
// #define DefineConvertFromMatrixXXI(type)                               \
//   class ConvertFromMatrix_##type : public NodeBase {                   \
//   public:                                                              \
//     ConvertFromMatrix_##type(NodeDescriptor const* descriptor, Index id) \
//         : NodeBase(descriptor, id) {}                                  \
//     static void register_this() {                                      \
//       NodeDescriptorFactory<ConvertFromMatrix_##type>()                \
//           .SetName("Convert_matxxi_to_" #type)                         \
//           .SetDescription("Convert a dynamic matrix to a " #type)      \
//           .AddInput<math::matxxi>("mat", "The matrix to convert")      \
//           .AddOutput<type>("out", "The converted value")               \
//           .FinalizeAndRegister();                                      \
//     }                                                                  \
//     void Apply(size_t) override {                                      \
//       auto* mat = RetriveInput<math::matxxi>(0);                       \
//       if (mat == nullptr) {                                            \
//         throw make_invalid_argument("Input is not set");               \
//       }                                                                \
//       Index rows = type ::RowsAtCompileTime;                             \
//       Index cols = type ::ColsAtCompileTime;                             \
//       if (mat->rows() != rows && rows != math::dynamic) {              \
//         throw make_invalid_argument("Rows mismatch");           \
//       } else if (mat->cols() != cols && cols != math::dynamic) {       \
//         throw make_invalid_argument("Cols mismatch");           \
//       }                                                                \
//       SetOutput<type>(0, *mat);                                        \
//     }                                                                  \
//   };
//
// #define DefineConvertToMatrixXXI(type)                                                             \
//   class ConvertToMatrix_##type : public NodeBase {                                                 \
//   public:                                                                                          \
//     ConvertToMatrix_##type(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {} \
//     static void register_this() {                                                                  \
//       NodeDescriptorFactory<ConvertToMatrix_##type>()                                              \
//           .SetName("Convert_" #type "_to_matxxi")                                                  \
//           .SetDescription("Convert a " #type " to a dynamic matrix")                               \
//           .AddInput<type>("in", "The value to convert")                                            \
//           .AddOutput<math::matxxi>("out", "The converted matrix")                                  \
//           .FinalizeAndRegister();                                                                  \
//     }                                                                                              \
//     void Apply(size_t) override {                                                                  \
//       auto* in = RetriveInput<type>(0);                                                            \
//       if (in == nullptr) {                                                                         \
//         throw make_invalid_argument("Input is not set");                                           \
//       }                                                                                            \
//       SetOutput<math::matxxi>(0, math::matxxi(*in));                                               \
//     }                                                                                              \
//   };
//
// using namespace ax::math;
// DefineConvertFromMatrixXXR(RealVector2);
// DefineConvertFromMatrixXXR(RealVector3);
// DefineConvertFromMatrixXXR(RealVector4);
// DefineConvertFromMatrixXXR(RealVectorX);
// DefineConvertFromMatrixXXR(RealField1);
// DefineConvertFromMatrixXXR(RealField2);
// DefineConvertFromMatrixXXR(RealField3);
// DefineConvertFromMatrixXXR(RealField4);
// DefineConvertFromMatrixXXI(IndexVec2);
// DefineConvertFromMatrixXXI(IndexVec3);
// DefineConvertFromMatrixXXI(IndexVec4);
// DefineConvertFromMatrixXXI(IndexVecX);
// DefineConvertFromMatrixXXI(IndexField1);
// DefineConvertFromMatrixXXI(IndexField2);
// DefineConvertFromMatrixXXI(IndexField3);
// DefineConvertFromMatrixXXI(IndexField4);
//
// DefineConvertToMatrixXXR(RealVector2);
// DefineConvertToMatrixXXR(RealVector3);
// DefineConvertToMatrixXXR(RealVector4);
// DefineConvertToMatrixXXR(RealVectorX);
// DefineConvertToMatrixXXR(RealField1);
// DefineConvertToMatrixXXR(RealField2);
// DefineConvertToMatrixXXR(RealField3);
// DefineConvertToMatrixXXR(RealField4);
//
// DefineConvertToMatrixXXI(IndexVec2);
// DefineConvertToMatrixXXI(IndexVec3);
// DefineConvertToMatrixXXI(IndexVec4);
// DefineConvertToMatrixXXI(IndexVecX);
// DefineConvertToMatrixXXI(IndexField1);
// DefineConvertToMatrixXXI(IndexField2);
// DefineConvertToMatrixXXI(IndexField3);
// DefineConvertToMatrixXXI(IndexField4);
//
// class Transpose_matxxr : public NodeBase {
// public:
//   Transpose_matxxr(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<Transpose_matxxr>()
//         .SetName("Transpose_matxxr")
//         .SetDescription("Transpose a matrix")
//         .AddInput<math::matxxr>("in", "The matrix to transpose")
//         .AddOutput<math::matxxr>("out", "The transposed matrix")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) override {
//     auto* in = RetriveInput<math::matxxr>(0);
//     if (in == nullptr) {
//       throw make_invalid_argument("Input is not set");
//     }
//     SetOutput<math::matxxr>(0, in->transpose());
//   }
// };
//
// class Transpose_matxxi : public NodeBase {
// public:
//   Transpose_matxxi(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//
//   static void register_this() {
//     NodeDescriptorFactory<Transpose_matxxi>()
//         .SetName("Transpose_matxxi")
//         .SetDescription("Transpose a matrix")
//         .AddInput<math::matxxi>("in", "The matrix to transpose")
//         .AddOutput<math::matxxi>("out", "The transposed matrix")
//         .FinalizeAndRegister();
//   }
//
//   void Apply(size_t) override {
//     auto* in = RetriveInput<math::matxxi>(0);
//     if (in == nullptr) {
//       throw make_invalid_argument("Input is not set");
//     }
//     SetOutput<math::matxxi>(0, in->transpose());
//   }
// };
//
// void register_math_types_nodes() {
//   InputColor3r::register_this();
//   InputColor4r::register_this();
//   Input_Vec2r::register_this();
//   Input_Vec3r::register_this();
//   Input_Vec4r::register_this();
//
//   RealVectorGetIndex<2>::register_this();
//   RealVectorGetIndex<3>::register_this();
//   RealVectorGetIndex<4>::register_this();
//
//   ExpandField<1, 2>::register_this();
//   ExpandField<1, 3>::register_this();
//   ExpandField<1, 4>::register_this();
//   ExpandField<2, 3>::register_this();
//   ExpandField<2, 4>::register_this();
//   ExpandField<3, 4>::register_this();
//
//   ElementWiseAffineTransform<1>::register_this();
//   ElementWiseAffineTransform<2>::register_this();
//   ElementWiseAffineTransform<3>::register_this();
//   ElementWiseAffineTransform<4>::register_this();
//
//   MakeConstantVector<2>::register_this();
//   MakeConstantVector<3>::register_this();
//   MakeConstantVector<4>::register_this();
//
//   MakeConstantField<2>::register_this();
//   MakeConstantField<3>::register_this();
//   MakeConstantField<4>::register_this();
//
//   MakeConstantMatrix<2, 2>::register_this();
//   MakeConstantMatrix<3, 3>::register_this();
//   MakeConstantMatrix<4, 4>::register_this();
//
//   MakeRandom_Real::register_this();
//   MakeRandom_Vector<2>::register_this();
//   MakeRandom_Vector<3>::register_this();
//   MakeRandom_Vector<4>::register_this();
//   MakeRandom_Matrix<2, 2>::register_this();
//   MakeRandom_Matrix<3, 3>::register_this();
//   MakeRandom_Matrix<4, 4>::register_this();
//
//   ConvertFromMatrix_RealVector2::register_this();
//   ConvertFromMatrix_RealVector3::register_this();
//   ConvertFromMatrix_RealVector4::register_this();
//   ConvertFromMatrix_RealVectorX::register_this();
//   ConvertFromMatrix_RealField1::register_this();
//   ConvertFromMatrix_RealField2::register_this();
//   ConvertFromMatrix_RealField3::register_this();
//   ConvertFromMatrix_RealField4::register_this();
//   ConvertFromMatrix_IndexVec2::register_this();
//   ConvertFromMatrix_IndexVec3::register_this();
//   ConvertFromMatrix_IndexVec4::register_this();
//   ConvertFromMatrix_IndexVecX::register_this();
//   ConvertFromMatrix_IndexField1::register_this();
//   ConvertFromMatrix_IndexField2::register_this();
//   ConvertFromMatrix_IndexField3::register_this();
//   ConvertFromMatrix_IndexField4::register_this();
//
//   ConvertToMatrix_RealVector2::register_this();
//   ConvertToMatrix_RealVector3::register_this();
//   ConvertToMatrix_RealVector4::register_this();
//   ConvertToMatrix_RealVectorX::register_this();
//   ConvertToMatrix_RealField1::register_this();
//   ConvertToMatrix_RealField2::register_this();
//   ConvertToMatrix_RealField3::register_this();
//   ConvertToMatrix_RealField4::register_this();
//   ConvertToMatrix_IndexVec2::register_this();
//   ConvertToMatrix_IndexVec3::register_this();
//   ConvertToMatrix_IndexVec4::register_this();
//   ConvertToMatrix_IndexVecX::register_this();
//   ConvertToMatrix_IndexField1::register_this();
//   ConvertToMatrix_IndexField2::register_this();
//   ConvertToMatrix_IndexField3::register_this();
//   ConvertToMatrix_IndexField4::register_this();
//
//   Transpose_matxxi::register_this();
//   Transpose_matxxr::register_this();
// }
//
// }  // namespace ax::nodes
