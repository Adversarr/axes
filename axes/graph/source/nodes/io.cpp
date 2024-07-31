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
  ReadObjNode(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ReadObjNode>()
        .SetName("Read_obj")
        .SetDescription("Reads an obj file and outputs a mesh")
        .AddInput<std::string>("file", "The path to the obj file")
        .AddInput<bool>("Reload Every Frame?", "If true, the obj file will be reloaded every frame")
        .AddOutput<geo::SurfaceMesh>("mesh", "The mesh read from the obj file")
        .FinalizeAndRegister();
  }

  void DoApply() {
    auto* file = RetriveInput<std::string>(0);
    if (file == nullptr) {
      throw make_invalid_argument("File path is not set");
    }

    try {
      auto mesh = geo::read_obj(*file);
      *RetriveOutput<geo::SurfaceMesh>(0) = std::move(mesh);
    } catch (...) {
      std::throw_with_nested(make_runtime_error("Read Obj failed"));
    }
  }

  void Apply(size_t frame_id) override {
    if (auto reload = RetriveInput<bool>(1); frame_id == 0 || (reload != nullptr && *reload)) {
      DoApply();
    }
  }

  void PreApply() override { DoApply(); }
};

class Selector_Mesh_Obj : public NodeBase {
public:
  Selector_Mesh_Obj(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Selector_Mesh_Obj>()
        .SetName("Selector_mesh_obj")
        .SetDescription("Select an asset from a list")
        .AddOutput<std::string>("mesh", "The mesh read from the obj file")
        .FinalizeAndRegister();

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

  void OnConstruct() override {
    assets_ = utils::discover_assets("/mesh/obj/");
    selected_idx_ = 0;
  }

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
  Selector_Mesh_Npy(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Selector_Mesh_Npy>()
        .SetName("Selector_mesh_npy")
        .SetDescription("Select an asset from a list")
        .AddOutput<std::string>("mesh", "The mesh read from the obj file")
        .FinalizeAndRegister();

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

  void OnConstruct() override {
    assets_ = utils::discover_assets("/mesh/npy/");
    selected_idx_ = 0;
  }

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
  ExportNumpy_matxxr(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  static void register_this() {
    NodeDescriptorFactory<ExportNumpy_matxxr>()
        .SetName("Write_npy_matxxr")
        .SetDescription("Exports a matxxr to a numpy file")
        .AddInput<matxxr>("data", "The matxxr to export")
        .AddInput<std ::string>("file", "The path to the numpy file")
        .FinalizeAndRegister();
  }
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
  ExportNumpy_matxxi(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  static void register_this() {
    NodeDescriptorFactory<ExportNumpy_matxxi>()
        .SetName("Write_npy_matxxr")
        .SetDescription("Exports a matxxr to a numpy file")
        .AddInput<matxxi>("data", "The matxxr to export")
        .AddInput<std::string>("file", "The path to the numpy file")
        .FinalizeAndRegister();
  }

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
  Read_npy_matxxr(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Read_npy_matxxr>()
        .SetName("Read_npy_matxxr")
        .SetDescription("Imports a numpy file to a matxxr")
        .AddInput<std::string>("file", "The path to the numpy file")
        .AddInput<bool>("reload", "Reload every frame")
        .AddOutput<math::matxxr>("out", "The matxxr read from the numpy file")
        .FinalizeAndRegister();
  }

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
  Read_npy_matxxi(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Read_npy_matxxi>()
        .SetName("Read_npy_matxxi")
        .SetDescription("Imports a numpy file to a matxxr")
        .AddInput<std::string>("file", "The path to the numpy file")
        .AddInput<bool>("reload", "Reload every frame")
        .AddOutput<math::matxxi>("out", "The matxxr read from the numpy file")
        .FinalizeAndRegister();
  }

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
  void PreApply() override {
    DoApply();
  }
};

class Read_SparseMatrix : public NodeBase {
public:
  Read_SparseMatrix(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Read_SparseMatrix>()
        .SetName("Read_SparseMatrix")
        .SetDescription("Reads a sparse matrix from a file")
        .AddInput<std::string>("file", "The path to the file")
        .AddInput<bool>("reload", "Reload every frame")
        .AddOutput<math::spmatr>("matrix", "The read sparse matrix")
        .FinalizeAndRegister();
  }

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

  void PreApply() override {
    DoApply();
  }
};

class Write_SparseMatrix : public NodeBase {
public:
  Write_SparseMatrix(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<Write_SparseMatrix>()
        .SetName("Write_SparseMatrix")
        .SetDescription("Writes a sparse matrix to a file")
        .AddInput<math::spmatr>("data", "The sparse matrix to write")
        .AddInput<std::string>("file", "The path to the file")
        .FinalizeAndRegister();
  }

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
