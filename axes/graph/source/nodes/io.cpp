#include "ax/nodes/io.hpp"

#include <imgui.h>
#include <imgui_node_editor.h>

#include <exception>

#include "ax/core/excepts.hpp"
#include "ax/geometry/common.hpp"
#include "ax/geometry/io.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/io.hpp"
#include "ax/utils/asset.hpp"

namespace ed = ax::NodeEditor;
using namespace ax;
using namespace graph;

namespace ax::nodes {

class ReadObjNode : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(ReadObjNode, "Read_obj", "Reads an obj file and outputs a mesh");
  AX_NODE_INPUTS((std::string, file, "The path to the obj file"),
                 (bool, reload, "If true, the obj file will be reloaded every frame"));
  AX_NODE_OUTPUTS((geo::SurfaceMesh, mesh, "The mesh read from the obj file"));

  void DoApply(bool do_not_throw) {
    auto const& file = AX_NODE_INPUT_ENSURE_EXTRACT(file);

    try {
      auto mesh = geo::read_obj(file);
      *RetriveOutput<geo::SurfaceMesh>(0) = std::move(mesh);
    } catch (...) {
      if (!do_not_throw) {
        std::throw_with_nested(make_runtime_error("Read Obj failed"));
      }
    }
  }

  void Apply(size_t frame_id) override {
    bool reload = AX_NODE_INPUT_EXTRACT_DEFAULT(reload, false);
    if (frame_id == 0 || reload) {
      DoApply(false);
    }
  }

  void PreApply() override { DoApply(true); }
};

class Selector_Mesh_Obj : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(Selector_Mesh_Obj, "Selector_mesh_obj", "Select an asset from a list",
                           register_render);
  AX_NODE_NO_INPUT();
  AX_NODE_OUTPUTS((std::string, mesh, "The mesh read from the obj file"));

  static void register_render() {
    add_custom_node_render(
        typeid(Selector_Mesh_Obj), CustomNodeRender{[](NodeBase* node) {
          begin_draw_node(node);
          draw_node_header_default(node);
          auto n = dynamic_cast<Selector_Mesh_Obj*>(node);
          ImGui::SetNextItemWidth(100);
          if (ImGui::Button("Select")) {
            ImGui::OpenPopup("Select Mesh");
          }
          ImGui::SameLine();
          ed::BeginPin(ed::PinId{n->GetOutputs()[0].id_}, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
          ImGui::Text("\"%s\"", n->assets_[n->selected_idx_].c_str());
          end_draw_node();

          ed::Suspend();
          ImGui::PushID(node);
          if (ImGui::BeginPopup("Select Mesh")) {
            for (int i = 0; i < (idx)n->assets_.size(); ++i) {
              if (ImGui::Selectable(n->assets_[i].c_str(), n->selected_idx_ == i)) {
                n->selected_idx_ = i;
              }
            }
            ImGui::EndPopup();
          }
          ImGui::PopID();
          ed::Resume();
        }});
  }

  void PreApply() override {
    *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_idx_]);
  }

  // TODO: ctor.
  // void OnConstruct() override {
  //   assets_ = utils::discover_assets("/mesh/obj/");
  //   selected_idx_ = 0;
  // }

  boost::json::object Serialize() const override {
    auto obj = NodeBase::Serialize();
    obj["selected_idx"] = selected_idx_;
    return obj;
  }

  void Deserialize(boost::json::object const& obj) override {
    if (obj.contains("selected_idx")) {
      selected_idx_ = obj.at("selected_idx").as_int64();
    }
  }

  std::vector<std::string> assets_;
  int selected_idx_;
};

class Selector_Mesh_Npy : public NodeBase {
public:
  AX_NODE_COMMON_WITH_CTOR(Selector_Mesh_Npy, "Selector_mesh_npy", "Select an asset from a list",
                           register_render);
  AX_NODE_NO_INPUT();
  AX_NODE_OUTPUTS((std::string, mesh, "The mesh read from the obj file"));

  static void register_render() {
    add_custom_node_render(
        typeid(Selector_Mesh_Npy), CustomNodeRender{[](NodeBase* node) {
          begin_draw_node(node);
          draw_node_header_default(node);
          auto n = dynamic_cast<Selector_Mesh_Npy*>(node);
          ImGui::SetNextItemWidth(100);
          if (ImGui::Button("Select")) {
            ImGui::OpenPopup("Select Mesh");
          }
          ImGui::SameLine();
          ed::BeginPin(ed::PinId{n->GetOutputs()[0].id_}, ed::PinKind::Output);
          ImGui::Text("%s", n->GetOutputs()[0].descriptor_->name_.c_str());
          ed::EndPin();
          ImGui::Text("\"%s\"", n->assets_[n->selected_idx_].c_str());
          end_draw_node();

          ed::Suspend();
          ImGui::PushID(node);
          if (ImGui::BeginPopup("Select Mesh")) {
            for (int i = 0; i < (idx)n->assets_.size(); ++i) {
              if (ImGui::Selectable(n->assets_[i].c_str(), n->selected_idx_ == i)) {
                n->selected_idx_ = i;
              }
            }
            ImGui::EndPopup();
          }
          ImGui::PopID();
          ed::Resume();
        }});
  }

  void PreApply() override {
    *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_idx_]);
  }

  // TODO: ctor.
  // void OnConstruct() override {
  //   assets_ = utils::discover_assets("/mesh/npy/");
  //   selected_idx_ = 0;
  // }

  boost::json::object Serialize() const override {
    auto obj = NodeBase::Serialize();
    obj["selected_idx"] = selected_idx_;
    return obj;
  }

  void Deserialize(boost::json::object const& obj) override {
    if (obj.contains("selected_idx")) {
      selected_idx_ = obj.at("selected_idx").as_int64();
    }
  }

  std::vector<std::string> assets_;
  int selected_idx_;
};

using namespace ax::math;

class ExportNumpy_matxxr : public NodeBase {
public:
  // ExportNumpy_matxxr(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  // static void register_this() {
  //   NodeDescriptorFactory<ExportNumpy_matxxr>()
  //       .SetName("Write_npy_matxxr")
  //       .SetDescription("Exports a matxxr to a numpy file")
  //       .AddInput<matxxr>("data", "The matxxr to export")
  //       .AddInput<std ::string>("file", "The path to the numpy file")
  //       .FinalizeAndRegister();
  // }

  AX_NODE_COMMON_WITH_CTOR(ExportNumpy_matxxr, "Write_npy_matxxr",
                           "Exports a matxxr to a numpy file");
  AX_NODE_INPUTS((matxxr, data, "The matxxr to export"),
                 (std::string, file, "The path to the numpy file"));
  AX_NODE_NO_OUTPUT();

  void Apply(size_t) override {
    auto* data = RetriveInput<matxxr>(0);
    auto* file = RetriveInput<std ::string>(1);
    if (data == nullptr) {
      throw make_invalid_argument("Data is not set");
    }
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }
    ax ::math ::matxxr out = *data;
    std ::string fname = *file;
    if (!fname.ends_with(".npy")) {
      fname += ".npy";
    }
    math::write_npy_v10(fname, out);
  }
};

class ExportNumpy_matxxi : public NodeBase {
public:
  // ExportNumpy_matxxi(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  // static void register_this() {
  //   NodeDescriptorFactory<ExportNumpy_matxxi>()
  //       .SetName("Write_npy_matxxr")
  //       .SetDescription("Exports a matxxr to a numpy file")
  //       .AddInput<matxxi>("data", "The matxxr to export")
  //       .AddInput<std::string>("file", "The path to the numpy file")
  //       .FinalizeAndRegister();
  // }

  AX_NODE_COMMON_WITH_CTOR(ExportNumpy_matxxi, "Write_npy_matxxr",
                           "Exports a matxxr to a numpy file");
  AX_NODE_INPUTS((matxxi, data, "The matxxr to export"),
                 (std::string, file, "The path to the numpy file"));
  AX_NODE_NO_OUTPUT();

  void Apply(size_t) override {
    auto* data = RetriveInput<matxxr>(0);
    auto* file = RetriveInput<std ::string>(1);
    if (data == nullptr) {
      throw make_invalid_argument("Data is not set");
    }
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }
    ax ::math ::matxxr out = *data;
    std ::string fname = *file;
    if (!fname.ends_with(".npy")) {
      fname += ".npy";
    }
    math::write_npy_v10(fname, out);
  }
};

class Read_npy_matxxr : public NodeBase {
public:
  // Read_npy_matxxr(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  //
  // static void register_this() {
  //   NodeDescriptorFactory<Read_npy_matxxr>()
  //       .SetName("Read_npy_matxxr")
  //       .SetDescription("Imports a numpy file to a matxxr")
  //       .AddInput<std::string>("file", "The path to the numpy file")
  //       .AddInput<bool>("reload", "Reload every frame")
  //       .AddOutput<math::matxxr>("out", "The matxxr read from the numpy file")
  //       .FinalizeAndRegister();
  // }
  AX_NODE_COMMON_WITH_CTOR(Read_npy_matxxr, "Read_npy_matxxr", "Imports a numpy file to a matxxr");
  AX_NODE_INPUTS((std::string, file, "The path to the numpy file"),
                 (bool, reload, "Reload every frame"));
  AX_NODE_OUTPUTS((matxxr, out, "The matxxr read from the numpy file"));

  size_t DoApply() {
    auto* file = RetriveInput<std::string>(0);
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }

    try {
      auto val = math::read_npy_v10_real(*file);
      SetOutput<matxxr>(0, val);
    } catch (...) {
      std::throw_with_nested(make_runtime_error("Read Npy failed"));
    }
  }

  void Apply(size_t frame_id) override {
    if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
      DoApply();
    }
  }

  void PreApply() override { DoApply(); }
};
class Read_npy_matxxi : public NodeBase {
public:
  // Read_npy_matxxi(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  //
  // static void register_this() {
  //   NodeDescriptorFactory<Read_npy_matxxi>()
  //       .SetName("Read_npy_matxxi")
  //       .SetDescription("Imports a numpy file to a matxxr")
  //       .AddInput<std::string>("file", "The path to the numpy file")
  //       .AddInput<bool>("reload", "Reload every frame")
  //       .AddOutput<math::matxxi>("out", "The matxxr read from the numpy file")
  //       .FinalizeAndRegister();
  // }
  AX_NODE_COMMON_WITH_CTOR(Read_npy_matxxi, "Read_npy_matxxi", "Imports a numpy file to a matxxr");
  AX_NODE_INPUTS((std::string, file, "The path to the numpy file"),
                 (bool, reload, "Reload every frame"));
  AX_NODE_OUTPUTS((matxxi, out, "The matxxr read from the numpy file"));

  void DoApply() {
    auto* file = RetriveInput<std::string>(0);
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }

    try {
      auto val = math::read_npy_v10_idx(*file);
      SetOutput<matxxi>(0, val);
    } catch (...) {
      std::throw_with_nested(make_runtime_error("Read Npy failed"));
    }
  }

  void Apply(size_t frame_id) override {
    if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
      DoApply();
    }
  }
  void PreApply() override { DoApply(); }
};

class Read_SparseMatrix : public NodeBase {
public:
  // Read_SparseMatrix(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  //
  // static void register_this() {
  //   NodeDescriptorFactory<Read_SparseMatrix>()
  //       .SetName("Read_SparseMatrix")
  //       .SetDescription("Reads a sparse matrix from a file")
  //       .AddInput<std::string>("file", "The path to the file")
  //       .AddInput<bool>("reload", "Reload every frame")
  //       .AddOutput<math::spmatr>("matrix", "The read sparse matrix")
  //       .FinalizeAndRegister();
  // }

  AX_NODE_COMMON_WITH_CTOR(Read_SparseMatrix, "Read_SparseMatrix",
                           "Reads a sparse matrix from a file");
  AX_NODE_INPUTS((std::string, file, "The path to the file"), (bool, reload, "Reload every frame"));
  AX_NODE_OUTPUTS((spmatr, matrix, "The read sparse matrix"));

  void DoApply() {
    auto* file = RetriveInput<std::string>(0);
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }

    try {
      auto val = math::read_sparse_matrix(*file);
      SetOutput<spmatr>(0, val);
    } catch (...) {
      std::throw_with_nested(make_runtime_error("Read SparseMatrix failed"));
    }
  }

  void Apply(size_t frame_id) override {
    if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
      DoApply();
    }
  }

  void PreApply() override { DoApply(); }
};

class Write_SparseMatrix : public NodeBase {
public:
  // Write_SparseMatrix(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  //
  // static void register_this() {
  //   NodeDescriptorFactory<Write_SparseMatrix>()
  //       .SetName("Write_SparseMatrix")
  //       .SetDescription("Writes a sparse matrix to a file")
  //       .AddInput<math::spmatr>("data", "The sparse matrix to write")
  //       .AddInput<std::string>("file", "The path to the file")
  //       .FinalizeAndRegister();
  // }

  AX_NODE_COMMON_WITH_CTOR(Write_SparseMatrix, "Write_SparseMatrix",
                           "Writes a sparse matrix to a file");
  AX_NODE_INPUTS((spmatr, data, "The sparse matrix to write"),
                 (std::string, file, "The path to the file"));
  AX_NODE_NO_OUTPUT();

  void Apply(size_t /* frame_id */) override {
    auto* file = RetriveInput<std::string>(1);
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }
    auto* matrix = RetriveInput<math::spmatr>(0);
    if (matrix == nullptr) {
      throw make_invalid_argument("Matrix is not set");
    }
    math::write_sparse_matrix(*file, *matrix);
  }
};

void register_io_nodes() {
  ReadObjNode::register_this();
  Selector_Mesh_Obj::register_this();
  Selector_Mesh_Npy::register_this();

  ExportNumpy_matxxr::register_this();
  ExportNumpy_matxxi::register_this();
  Read_npy_matxxr::register_this();
  Read_npy_matxxi::register_this();

  Write_SparseMatrix::register_this();
  Read_SparseMatrix::register_this();
}
}  // namespace ax::nodes
