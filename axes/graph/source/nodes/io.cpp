#include "ax/nodes/io.hpp"

#include "ax/geometry/common.hpp"
#include "ax/geometry/io.hpp"
#include "ax/utils/asset.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/render.hpp"
#include "ax/utils/status.hpp"
#include "ax/math/io.hpp"
#include <imgui.h>
#include <imgui_node_editor.h>

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

  Status Apply(idx frame_id) {
    auto* file = RetriveInput<std::string>(0);
    auto* reload = RetriveInput<bool>(1);
    if (file == nullptr) {
      return utils::FailedPreconditionError("File path is not set");
    }

    bool need_reload_this_frame;
    if (reload == nullptr) {
      need_reload_this_frame = frame_id == 0;
    } else {
      need_reload_this_frame = *reload;
    }

    if (need_reload_this_frame) {
      auto mesh = geo::read_obj(*file);
      if (!mesh.ok()) {
        AX_LOG(ERROR) << "Failed to read obj file: " << file->c_str();
        return mesh.status();
      }
      *RetriveOutput<geo::SurfaceMesh>(0) = std::move(mesh.value());
    }

    AX_RETURN_OK();
  }
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

    add_custom_node_render(typeid(Selector_Mesh_Obj), CustomNodeRender{[](NodeBase* node) {
      begin_draw_node(node);draw_node_header_default(node);
        auto n = dynamic_cast<Selector_Mesh_Obj*>(node);
        ImGui::SetNextItemWidth(100);
        if (ImGui::Button("Select")) {
          ImGui::OpenPopup("Select Mesh");
        }
        ImGui::SameLine();
        ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
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

  Status PreApply() override {
    *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_idx_]);
    AX_RETURN_OK();
  }

  Status OnConstruct() override {
    assets_ = utils::discover_assets("/mesh/obj/");
    selected_idx_ = 0;
    AX_RETURN_OK();
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

    add_custom_node_render(typeid(Selector_Mesh_Npy), CustomNodeRender{[](NodeBase* node) {
      begin_draw_node(node);draw_node_header_default(node);
        auto n = dynamic_cast<Selector_Mesh_Npy*>(node);
        ImGui::SetNextItemWidth(100);
        if (ImGui::Button("Select")) {
          ImGui::OpenPopup("Select Mesh");
        }
        ImGui::SameLine();
        ed::BeginPin(n->GetOutputs()[0].id_, ed::PinKind::Output);
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

  Status PreApply() override {
    *RetriveOutput<std::string>(0) = utils::get_asset(assets_[selected_idx_]);
    AX_RETURN_OK();
  }

  Status OnConstruct() override {
    assets_ = utils::discover_assets("/mesh/npy/");
    selected_idx_ = 0;
    AX_RETURN_OK();
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
        .SetName("Export_numpy_matxxr")
        .SetDescription("Exports a matxxr to a numpy file")
        .AddInput<matxxr>("data", "The matxxr to export")
        .AddInput<std ::string>("file", "The path to the numpy file")
        .FinalizeAndRegister();
  }
  Status Apply(idx) override {
    auto *data = RetriveInput<matxxr>(0);
    auto *file = RetriveInput<std ::string>(1);
    if (data == nullptr) {
      return utils ::FailedPreconditionError("Data is not set");
    }
    if (file == nullptr) {
      return utils ::FailedPreconditionError("File path is not set");
    }
    ax ::math ::matxxr out = *data;
    std ::string fname = *file;
    if (!fname.ends_with(".npy")) {
      fname += ".npy";
    }
    auto status = math ::write_npy_v10(fname, out);
    if (!status.ok()) {
      return status;
    }
    return ::ax ::utils ::OkStatus();
  }
};

class ExportNumpy_matxxi : public NodeBase {
public:
  ExportNumpy_matxxi(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}
  static void register_this() {
    NodeDescriptorFactory<ExportNumpy_matxxi>()
        .SetName("Export_numpy_matxxr")
        .SetDescription("Exports a matxxr to a numpy file")
        .AddInput<matxxi>("data", "The matxxr to export")
        .AddInput<std::string>("file", "The path to the numpy file")
        .FinalizeAndRegister();
  }

  Status Apply(idx) override {
    auto *data = RetriveInput<matxxr>(0);
    auto *file = RetriveInput<std ::string>(1);
    if (data == nullptr) {
      return utils ::FailedPreconditionError("Data is not set");
    }
    if (file == nullptr) {
      return utils ::FailedPreconditionError("File path is not set");
    }
    ax ::math ::matxxr out = *data;
    std ::string fname = *file;
    if (!fname.ends_with(".npy")) {
      fname += ".npy";
    }
    auto status = math ::write_npy_v10(fname, out);
    if (!status.ok()) {
      return status;
    }
    return ::ax ::utils ::OkStatus();
  }
};

class ImportNumpy_Matxxr : public NodeBase {
public:
  ImportNumpy_Matxxr(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ImportNumpy_Matxxr>()
        .SetName("Import_numpy_matxxr")
        .SetDescription("Imports a numpy file to a matxxr")
        .AddInput<std::string>("file", "The path to the numpy file")
        .AddOutput<math::matxxr>("data", "The matxxr read from the numpy file")
        .FinalizeAndRegister();
  }

  Status Apply(idx /* frame_id */) override {
    auto* file = RetriveInput<std::string>(0);
    if (file == nullptr) {
      return utils::FailedPreconditionError("File path is not set");
    }
    auto status = math::read_npy_v10_real(*file);
    if (!status.ok()) {
      return status.status();
    }
    SetOutput<matxxr>(0, std::move(status.value()));
    AX_RETURN_OK();
  }
};
class ImportNumpy_Matxxi : public NodeBase {
public:
  ImportNumpy_Matxxi(NodeDescriptor const* descriptor, idx id) : NodeBase(descriptor, id) {}

  static void register_this() {
    NodeDescriptorFactory<ImportNumpy_Matxxi>()
        .SetName("Import_numpy_matxxi")
        .SetDescription("Imports a numpy file to a matxxr")
        .AddInput<std::string>("file", "The path to the numpy file")
        .AddOutput<math::matxxi>("data", "The matxxr read from the numpy file")
        .FinalizeAndRegister();
  }

  Status Apply(idx /* frame_id */) override {
    auto* file = RetriveInput<std::string>(0);
    if (file == nullptr) {
      return utils::FailedPreconditionError("File path is not set");
    }
    auto status = math::read_npy_v10_idx(*file);
    if (!status.ok()) {
      return status.status();
    }
    SetOutput<matxxi>(0, std::move(status.value()));
    AX_RETURN_OK();
  }
};

void register_io_nodes() {
  ReadObjNode::register_this();
  Selector_Mesh_Obj::register_this();
  Selector_Mesh_Npy::register_this();

  ExportNumpy_matxxr::register_this();
  ExportNumpy_matxxi::register_this();
  ImportNumpy_Matxxr::register_this();
  ImportNumpy_Matxxi::register_this();
}
}  // namespace ax::nodes
