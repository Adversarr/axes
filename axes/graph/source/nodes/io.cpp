#include "ax/geometry/io.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>
#include <range/v3/view/zip.hpp>
#include <exception>

#include "ax/core/excepts.hpp"
#include "ax/core/logging.hpp"
#include "ax/geometry/common.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/io.hpp"

#include "ax/math/accessor.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/ranges.hpp"

namespace ed = ax::NodeEditor;
using namespace ax;
using namespace graph;

namespace ax::nodes {
class Load_Obj final : public NodeDerive<Load_Obj> {
public:
  CG_NODE_COMMON(Load_Obj, "Load/Obj", "Loads a mesh from an obj file");
  CG_NODE_INPUTS((std::string, file, "The path to the obj file"),
                 (bool, flip_yz, "If true, the y and z axis will be flipped (e.g. blender)", true));
  CG_NODE_OUTPUTS((geo::SurfaceMesh, mesh, "The mesh read from the obj file"));

  static void RenderWidget(NodeBase* ptr) {
    static_cast<Load_Obj*>(ptr)->RenderWidgetThis();
  }

  void RenderWidgetThis() noexcept {
    ImGui::Text("Is loaded: %s", is_loaded_ ? "true" : "false");
    ImGui::Text("Mesh file: \"%s\"", last_input_.c_str());
    ImGui::PushID(this);
    if (ImGui::Button("Reload!")) {
      try {
        DoLoading();
      } catch (std::exception const& e) {
        AX_ERROR("Failed reload: {}", e.what());
      }
    }
    ImGui::PopID();
  }

  static void OnRegister() {
    add_custom_node_widget(name(), {RenderWidget});
  }

  void DoLoading() {
    std::string const &file_name = Ensure(in::file);
    bool flip_yz = GetOr(in::flip_yz);
    if (file_name != last_input_) {
      is_loaded_ = false;
      last_input_ = file_name;
      geo::SurfaceMesh mesh = geo::read_obj(file_name);
      if (flip_yz) {
        auto vert = math::make_accessor(mesh.vertices_);
        auto ind = math::make_accessor(mesh.indices_);
        for (auto v: vert) {
          std::swap(v.y(), v.z());
        }
        for (auto i: ind) {
          std::swap(i.y(), i.z());
        }
      }
      // Successfully loaded, set load
      is_loaded_ = true;
      SetAll(std::move(mesh));
    }
  }

  void OnConnectDispatch(in::file_) try {
    DoLoading();
  } catch (...) {
  }

  void operator()(Context &ctx) override {
    DoLoading();
  }

  std::string last_input_;
  bool is_loaded_ = false;
};

class Asset_Selector_MeshObj final : public NodeDerive<Asset_Selector_MeshObj> {
public:
  CG_NODE_COMMON(Asset_Selector_MeshObj, "Asset/Selector/MeshObj",
                 "Select an asset from a mesh/obj");
  CG_NODE_INPUTS();

  CG_NODE_OUTPUTS((std::string, asset, "The selected asset"));

  static void RenderThis(NodeBase *base) {
    auto *node = static_cast<Asset_Selector_MeshObj *>(base);
    begin_draw_node(node);
    draw_node_header_default(node);
    draw_node_content_default(node);
    if (!all_obj_files_in_asset_.empty()) {
      ImGui::Text("Selected: %s", all_obj_files_in_asset_[node->selected_].c_str());
    }
    end_draw_node();
  }

  static void RenderThisWidget(NodeBase *base) {
    auto *node = static_cast<Asset_Selector_MeshObj *>(base);
    ImGui::BeginChild("Asset Selector");
    ImGui::PushID(node);
    bool changed = false;
    if (ImGui::BeginCombo("##Asset Selector", all_obj_files_in_asset_[node->selected_].c_str())) {
      for (size_t i = 0; i < all_obj_files_in_asset_.size(); ++i) {
        bool is_selected = (node->selected_ == i);
        if (ImGui::Selectable(all_obj_files_in_asset_[i].c_str(), is_selected)) {
          node->selected_ = i;
          changed = true;
        }

        if (is_selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    if (changed) {
      node->SetOutput();
    }
    ImGui::PopID();
    ImGui::EndChild();
  }

  static void OnRegister() {
    add_custom_node_widget(name(), {RenderThisWidget});
    add_custom_node_render(name(), {RenderThis});
    Reload();
  }

  void OnConstruct() {
    SetOutput();
  }

  void SetOutput() {if (all_obj_files_in_asset_.empty()) {
      AX_WARN("No obj files found in asset '/mesh/obj/'!");
      SetAll("");
    } else {
      Set(out::asset, utils::get_asset(all_obj_files_in_asset_[selected_]));
    }}

  void operator()(Context &ctx) override {
    SetOutput();
  }

  static void Reload() { all_obj_files_in_asset_ = utils::discover_assets("/mesh/obj/"); }

  inline static std::vector<std::string> all_obj_files_in_asset_;
  size_t selected_ = 0;
};

void register_io(NodeRegistry &reg) {
  Asset_Selector_MeshObj::RegisterTo(reg);
  Load_Obj::RegisterTo(reg);
}
}  // namespace ax::nodes

 // namespace ax::nodes {
 //
 // class ReadObjNode : public NodeBase {
 // public:
 //   AX_NODE_COMMON_WITH_CTOR(ReadObjNode, "Read_obj", "Reads an obj file and outputs a mesh");
 //   AX_NODE_INPUTS((std::string, file, "The path to the obj file"),
 //                  (bool, reload, "If true, the obj file will be reloaded every frame"));
 //   AX_NODE_OUTPUTS((geo::SurfaceMesh, mesh, "The mesh read from the obj file"));
 //
 //   void DoApply(bool do_not_throw) {
 //     auto const& file = AX_NODE_INPUT_ENSURE_EXTRACT(file);
 //
 //     try {
 //       auto mesh = geo::read_obj(file);
 //       *RetriveOutput<geo::SurfaceMesh>(0) = std::move(mesh);
 //     } catch (...) {
 //       if (!do_not_throw) {
 //         std::throw_with_nested(make_runtime_error("Read Obj failed"));
 //       }
 //     }
 //   }
 //
 //   void Apply(size_t frame_id) override {
 //     bool reload = AX_NODE_INPUT_EXTRACT_DEFAULT(reload, false);
 //     if (frame_id == 0 || reload) {
 //       DoApply(false);
 //     }
 //   }

//   void PreApply() override { DoApply(true); }
// };
//
// class Selector_Mesh_Obj : public NodeBase {
// public:
//   AX_NODE_COMMON_WITH_CTOR(Selector_Mesh_Obj, "Selector_mesh_obj", "Select an asset from a list",
//                            register_render);
//   AX_NODE_NO_INPUT();
//   AX_NODE_OUTPUTS((std::string, mesh, "The mesh read from the obj file"));
//
//   static void register_render() {
//     add_custom_node_render(
//         typeid(Selector_Mesh_Obj), CustomNodeRender{[](NodeBase* node) {
//           begin_draw_node(node);
//           draw_node_header_default(node);
//           auto n = dynamic_cast<Selector_Mesh_Obj*>(node);
//           ImGui::SetNextItemWidth(100);
//           if (ImGui::Button("Select")) {
//             ImGui::OpenPopup("Select Mesh");
//           }
//           ImGui::SameLine();
//           ed::BeginPin(ed::PinId{n->GetOutputs()[0].id_}, ed::PinKind::Output);
//           ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//           ed::EndPin();
//           ImGui::Text("\"%s\"", n->assets_[n->selected_Index_].c_str());
//           end_draw_node();
//
//           ed::Suspend();
//           ImGui::PushID(node);
//           if (ImGui::BeginPopup("Select Mesh")) {
//             for (int i = 0; i < (Index)n->assets_.size(); ++i) {
//               if (ImGui::Selectable(n->assets_[i].c_str(), n->selected_Index_ == i)) {
//                 n->selected_Index_ = i;
//               }
//             }
//             ImGui::EndPopup();
//           }
//           ImGui::PopID();
//           ed::Resume();
//         }});
//   }
//
//   void PreApply() override {
//     *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_Index_]);
//   }
//
//   // TODO: ctor.
//   // void OnConstruct() override {
//   //   assets_ = utils::discover_assets("/mesh/obj/");
//   //   selected_Index_ = 0;
//   // }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     obj["selected_Index"] = selected_Index_;
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     if (obj.contains("selected_Index")) {
//       selected_Index_ = obj.at("selected_Index").as_int64();
//     }
//   }
//
//   std::vector<std::string> assets_;
//   int selected_Index_;
// };
//
// class Selector_Mesh_Npy : public NodeBase {
// public:
//   AX_NODE_COMMON_WITH_CTOR(Selector_Mesh_Npy, "Selector_mesh_npy", "Select an asset from a list",
//                            register_render);
//   AX_NODE_NO_INPUT();
//   AX_NODE_OUTPUTS((std::string, mesh, "The mesh read from the obj file"));
//
//   static void register_render() {
//     add_custom_node_render(
//         typeid(Selector_Mesh_Npy), CustomNodeRender{[](NodeBase* node) {
//           begin_draw_node(node);
//           draw_node_header_default(node);
//           auto n = dynamic_cast<Selector_Mesh_Npy*>(node);
//           ImGui::SetNextItemWidth(100);
//           if (ImGui::Button("Select")) {
//             ImGui::OpenPopup("Select Mesh");
//           }
//           ImGui::SameLine();
//           ed::BeginPin(ed::PinId{n->GetOutputs()[0].id_}, ed::PinKind::Output);
//           ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
//           ed::EndPin();
//           ImGui::Text("\"%s\"", n->assets_[n->selected_Index_].c_str());
//           end_draw_node();
//
//           ed::Suspend();
//           ImGui::PushID(node);
//           if (ImGui::BeginPopup("Select Mesh")) {
//             for (int i = 0; i < (Index)n->assets_.size(); ++i) {
//               if (ImGui::Selectable(n->assets_[i].c_str(), n->selected_Index_ == i)) {
//                 n->selected_Index_ = i;
//               }
//             }
//             ImGui::EndPopup();
//           }
//           ImGui::PopID();
//           ed::Resume();
//         }});
//   }
//
//   void PreApply() override {
//     *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_Index_]);
//   }
//
//   // TODO: ctor.
//   // void OnConstruct() override {
//   //   assets_ = utils::discover_assets("/mesh/npy/");
//   //   selected_Index_ = 0;
//   // }
//
//   boost::json::object Serialize() const override {
//     auto obj = NodeBase::Serialize();
//     obj["selected_Index"] = selected_Index_;
//     return obj;
//   }
//
//   void Deserialize(boost::json::object const& obj) override {
//     if (obj.contains("selected_Index")) {
//       selected_Index_ = obj.at("selected_Index").as_int64();
//     }
//   }
//
//   std::vector<std::string> assets_;
//   int selected_Index_;
// };
//
// using namespace ax::math;
//
// class ExportNumpy_RealMatrixX : public NodeBase {
// public:
//   // ExportNumpy_RealMatrixX(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor,
//   id) {}
//   // static void register_this() {
//   //   NodeDescriptorFactory<ExportNumpy_RealMatrixX>()
//   //       .SetName("Write_npy_RealMatrixX")
//   //       .SetDescription("Exports a RealMatrixX to a numpy file")
//   //       .AddInput<RealMatrixX>("data", "The RealMatrixX to export")
//   //       .AddInput<std ::string>("file", "The path to the numpy file")
//   //       .FinalizeAndRegister();
//   // }
//
//   AX_NODE_COMMON_WITH_CTOR(ExportNumpy_RealMatrixX, "Write_npy_RealMatrixX",
//                            "Exports a RealMatrixX to a numpy file");
//   AX_NODE_INPUTS((RealMatrixX, data, "The RealMatrixX to export"),
//                  (std::string, file, "The path to the numpy file"));
//   AX_NODE_NO_OUTPUT();
//
//   void Apply(size_t) override {
//     auto* data = RetriveInput<RealMatrixX>(0);
//     auto* file = RetriveInput<std ::string>(1);
//     if (data == nullptr) {
//       throw make_invalid_argument("Data is not set");
//     }
//     if (file == nullptr) {
//       throw make_invalid_argument("File path is not set");
//     }
//     ax ::math ::RealMatrixX out = *data;
//     std ::string fname = *file;
//     if (!fname.ends_with(".npy")) {
//       fname += ".npy";
//     }
//     math::write_npy_v10(fname, out);
//   }
// };
//
// class ExportNumpy_IndexMatrixX : public NodeBase {
// public:
//   // ExportNumpy_IndexMatrixX(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor,
//   id) {}
//   // static void register_this() {
//   //   NodeDescriptorFactory<ExportNumpy_IndexMatrixX>()
//   //       .SetName("Write_npy_RealMatrixX")
//   //       .SetDescription("Exports a RealMatrixX to a numpy file")
//   //       .AddInput<IndexMatrixX>("data", "The RealMatrixX to export")
//   //       .AddInput<std::string>("file", "The path to the numpy file")
//   //       .FinalizeAndRegister();
//   // }
//
//   AX_NODE_COMMON_WITH_CTOR(ExportNumpy_IndexMatrixX, "Write_npy_RealMatrixX",
//                            "Exports a RealMatrixX to a numpy file");
//   AX_NODE_INPUTS((IndexMatrixX, data, "The RealMatrixX to export"),
//                  (std::string, file, "The path to the numpy file"));
//   AX_NODE_NO_OUTPUT();
//
//   void Apply(size_t) override {
//     auto* data = RetriveInput<RealMatrixX>(0);
//     auto* file = RetriveInput<std ::string>(1);
//     if (data == nullptr) {
//       throw make_invalid_argument("Data is not set");
//     }
//     if (file == nullptr) {
//       throw make_invalid_argument("File path is not set");
//     }
//     ax ::math ::RealMatrixX out = *data;
//     std ::string fname = *file;
//     if (!fname.ends_with(".npy")) {
//       fname += ".npy";
//     }
//     math::write_npy_v10(fname, out);
//   }
// };
//
// class Read_npy_RealMatrixX : public NodeBase {
// public:
//   // Read_npy_RealMatrixX(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id)
//   {}
//   //
//   // static void register_this() {
//   //   NodeDescriptorFactory<Read_npy_RealMatrixX>()
//   //       .SetName("Read_npy_RealMatrixX")
//   //       .SetDescription("Imports a numpy file to a RealMatrixX")
//   //       .AddInput<std::string>("file", "The path to the numpy file")
//   //       .AddInput<bool>("reload", "Reload every frame")
//   //       .AddOutput<math::RealMatrixX>("out", "The RealMatrixX read from the numpy file")
//   //       .FinalizeAndRegister();
//   // }
//   AX_NODE_COMMON_WITH_CTOR(Read_npy_RealMatrixX, "Read_npy_RealMatrixX", "Imports a numpy file to
//   a RealMatrixX"); AX_NODE_INPUTS((std::string, file, "The path to the numpy file"),
//                  (bool, reload, "Reload every frame"));
//   AX_NODE_OUTPUTS((RealMatrixX, out, "The RealMatrixX read from the numpy file"));
//
//   size_t DoApply() {
//     auto* file = RetriveInput<std::string>(0);
//     if (file == nullptr) {
//       throw make_invalid_argument("File path is not set");
//     }
//
//     try {
//       auto val = math::read_npy_v10_real(*file);
//       SetOutput<RealMatrixX>(0, val);
//     } catch (...) {
//       std::throw_with_nested(make_runtime_error("Read Npy failed"));
//     }
//   }
//
//   void Apply(size_t frame_id) override {
//     if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
//       DoApply();
//     }
//   }
//
//   void PreApply() override { DoApply(); }
// };
// class Read_npy_IndexMatrixX : public NodeBase {
// public:
//   // Read_npy_IndexMatrixX(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id)
//   {}
//   //
//   // static void register_this() {
//   //   NodeDescriptorFactory<Read_npy_IndexMatrixX>()
//   //       .SetName("Read_npy_IndexMatrixX")
//   //       .SetDescription("Imports a numpy file to a RealMatrixX")
//   //       .AddInput<std::string>("file", "The path to the numpy file")
//   //       .AddInput<bool>("reload", "Reload every frame")
//   //       .AddOutput<math::IndexMatrixX>("out", "The RealMatrixX read from the numpy file")
//   //       .FinalizeAndRegister();
//   // }
//   AX_NODE_COMMON_WITH_CTOR(Read_npy_IndexMatrixX, "Read_npy_IndexMatrixX", "Imports a numpy file
//   to a RealMatrixX"); AX_NODE_INPUTS((std::string, file, "The path to the numpy file"),
//                  (bool, reload, "Reload every frame"));
//   AX_NODE_OUTPUTS((IndexMatrixX, out, "The RealMatrixX read from the numpy file"));
//
//   void DoApply() {
//     auto* file = RetriveInput<std::string>(0);
//     if (file == nullptr) {
//       throw make_invalid_argument("File path is not set");
//     }
//
//     try {
//       auto val = math::read_npy_v10_Index(*file);
//       SetOutput<IndexMatrixX>(0, val);
//     } catch (...) {
//       std::throw_with_nested(make_runtime_error("Read Npy failed"));
//     }
//   }
//
//   void Apply(size_t frame_id) override {
//     if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
//       DoApply();
//     }
//   }
//   void PreApply() override { DoApply(); }
// };
//
// class Read_SparseMatrix : public NodeBase {
// public:
//   // Read_SparseMatrix(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//   //
//   // static void register_this() {
//   //   NodeDescriptorFactory<Read_SparseMatrix>()
//   //       .SetName("Read_SparseMatrix")
//   //       .SetDescription("Reads a sparse matrix from a file")
//   //       .AddInput<std::string>("file", "The path to the file")
//   //       .AddInput<bool>("reload", "Reload every frame")
//   //       .AddOutput<math::RealSparseMatrix>("matrix", "The read sparse matrix")
//   //       .FinalizeAndRegister();
//   // }
//
//   AX_NODE_COMMON_WITH_CTOR(Read_SparseMatrix, "Read_SparseMatrix",
//                            "Reads a sparse matrix from a file");
//   AX_NODE_INPUTS((std::string, file, "The path to the file"), (bool, reload, "Reload every
//   frame")); AX_NODE_OUTPUTS((RealSparseMatrix, matrix, "The read sparse matrix"));
//
//   void DoApply() {
//     auto* file = RetriveInput<std::string>(0);
//     if (file == nullptr) {
//       throw make_invalid_argument("File path is not set");
//     }
//
//     try {
//       auto val = math::read_sparse_matrix(*file);
//       SetOutput<RealSparseMatrix>(0, val);
//     } catch (...) {
//       std::throw_with_nested(make_runtime_error("Read SparseMatrix failed"));
//     }
//   }
//
//   void Apply(size_t frame_id) override {
//     if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
//       DoApply();
//     }
//   }
//
//   void PreApply() override { DoApply(); }
// };
//
// class Write_SparseMatrix : public NodeBase {
// public:
//   // Write_SparseMatrix(NodeDescriptor const* descriptor, Index id) : NodeBase(descriptor, id) {}
//   //
//   // static void register_this() {
//   //   NodeDescriptorFactory<Write_SparseMatrix>()
//   //       .SetName("Write_SparseMatrix")
//   //       .SetDescription("Writes a sparse matrix to a file")
//   //       .AddInput<math::RealSparseMatrix>("data", "The sparse matrix to write")
//   //       .AddInput<std::string>("file", "The path to the file")
//   //       .FinalizeAndRegister();
//   // }
//
//   AX_NODE_COMMON_WITH_CTOR(Write_SparseMatrix, "Write_SparseMatrix",
//                            "Writes a sparse matrix to a file");
//   AX_NODE_INPUTS((RealSparseMatrix, data, "The sparse matrix to write"),
//                  (std::string, file, "The path to the file"));
//   AX_NODE_NO_OUTPUT();
//
//   void Apply(size_t /* frame_id */) override {
//     auto* file = RetriveInput<std::string>(1);
//     if (file == nullptr) {
//       throw make_invalid_argument("File path is not set");
//     }
//     auto* matrix = RetriveInput<math::RealSparseMatrix>(0);
//     if (matrix == nullptr) {
//       throw make_invalid_argument("Matrix is not set");
//     }
//     math::write_sparse_matrix(*file, *matrix);
//   }
// };
//
// void register_io_nodes() {
//   ReadObjNode::register_this();
//   Selector_Mesh_Obj::register_this();
//   Selector_Mesh_Npy::register_this();
//
//   ExportNumpy_RealMatrixX::register_this();
//   ExportNumpy_IndexMatrixX::register_this();
//   Read_npy_RealMatrixX::register_this();
//   Read_npy_IndexMatrixX::register_this();
//
//   Write_SparseMatrix::register_this();
//   Read_SparseMatrix::register_this();
// }

// }  // namespace ax::nodes