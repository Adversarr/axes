#include "ax/graph/render.hpp"

#include <imnode/imgui_canvas.h>
#include <imnode/imgui_node_editor.h>

#include "ax/core/entt.hpp"
#include "ax/gl/context.hpp"
#include "ax/graph/executor.hpp"
#include "ax/graph/graph.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/serial.hpp"
#include "ax/utils/status.hpp"

namespace ed = ax::NodeEditor;

static bool start_with(std::string const& str, const char* prefix, size_t size) {
  return str.size() >= size && std::equal(prefix, prefix + size, str.begin(), [](char a, char b) {
           return std::tolower(a) == std::tolower(b);
         });
}

namespace ax::graph {

///////////////////////////////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////////////////////////////

GraphRendererOptions opt_;
ed::EditorContext* context_;
ed::PinId hovered_pin_;

void draw_node(NodeBase* node) {
  auto const& in = node->GetInputs();
  auto const& out = node->GetOutputs();
  size_t n_max_io = std::max(in.size(), out.size());
  size_t n_max_name_size = 0;
  std::for_each(in.begin(), in.end(), [&n_max_name_size](Pin const& p) {
    n_max_name_size = std::max(n_max_name_size, p.descriptor_->name_.size());
  });
  std::for_each(out.begin(), out.end(), [&n_max_name_size](Pin const& p) {
    n_max_name_size = std::max(n_max_name_size, p.descriptor_->name_.size());
  });

  ed::BeginNode(node->GetId());
  ImGui::Text("## %ld: %s", node->GetId(), node->GetName().c_str());

  if (auto render = get_custom_node_render(node->GetType()); render) {
    render->widget_(node);
  } else {
    for (size_t i = 0; i < n_max_io; ++i) {
      if (i < in.size()) {
        ed::BeginPin(in[i].id_, ed::PinKind::Input);
        std::string spacing(n_max_name_size - in[i].descriptor_->name_.size(), ' ');
        // ed::PinPivotAlignment(ImVec2(0, 0.5));
        ImGui::Text("-> %s %s", in[i].descriptor_->name_.c_str(), spacing.c_str());
        ImGui::SameLine();
        ed::EndPin();
      } else {
        std::string spacing(n_max_name_size, ' ');
        ImGui::Text("    %s", spacing.c_str());
      }
      ImGui::SameLine();
      if (i < out.size()) {
        std::string spacing(n_max_name_size - out[i].descriptor_->name_.size(), ' ');
        ed::BeginPin(out[i].id_, ed::PinKind::Output);
        ImGui::Indent();
        ImGui::Text("%s %s ->", spacing.c_str(), out[i].descriptor_->name_.c_str());
        ImGui::Unindent();
        ed::EndPin();
      } else {
        ImGui::Text("    ");
      }
    }
  }
  NodeEditor::EndNode();
}

void draw_socket(Socket* socket) {
  ed::Link(socket->id_, socket->input_->id_, socket->output_->id_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Handling User Input
///////////////////////////////////////////////////////////////////////////////////////////////////
void handle_inputs() {
  auto& g = ensure_resource<Graph>();
  if (ed::BeginCreate()) {
    ed::PinId input_id, output_id;
    if (ed::QueryNewLink(&input_id, &output_id)) {
      if (ed::AcceptNewItem()) {
        idx inid = input_id.Get();
        idx outid = output_id.Get();
        auto input = g.GetPin(inid);
        auto output = g.GetPin(outid);
        StatusOr<Socket*> sock;
        if (input->is_input_) {
          sock = g.AddSocket(output, input);
        } else {
          sock = g.AddSocket(input, output);
        }
        if (sock.ok()) {
          auto s = sock.value();
          AX_LOG(INFO) << "Created link: " << s->input_->node_id_ << ":"
                       << s->input_->node_io_index_ << "->" << s->output_->node_id_ << ":"
                       << s->output_->node_io_index_;
          draw_socket(s);
        }
      }
    }
  }
  ed::EndCreate();

  if (ed::BeginDelete()) {
    ed::LinkId link_id = 0;
    while (ed::QueryDeletedLink(&link_id)) {
      if (ed::AcceptDeletedItem()) {
        AX_LOG(INFO) << "Deleted link: " << link_id.Get();
        g.RemoveSocket(link_id.Get());
      }
    }

    ed::NodeId node_id = 0;
    while (ed::QueryDeletedNode(&node_id)) {
      if (ed::AcceptDeletedItem()) {
        AX_LOG(INFO) << "Deleted node: " << node_id.Get();
        g.RemoveNode(node_id.Get());
      }
    }
  }
  ed::EndDelete();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Handle User Selection
///////////////////////////////////////////////////////////////////////////////////////////////////
void handle_selection() {
  auto& g = ensure_resource<Graph>();
  ed::NodeId nodes[1];
  ed::LinkId links[1];
  ed::GetSelectedNodes(nodes, 1);
  ed::GetSelectedLinks(links, 1);
  ed::NodeId select_node0 = nodes[0];
  ed::LinkId select_link0 = links[0];
  hovered_pin_ = ed::GetHoveredPin();

  ed::Suspend();
  if (ed::ShowNodeContextMenu(&select_node0)) {
    ImGui::OpenPopup("Node Context Menu");
  } else if (ed::ShowLinkContextMenu(&select_link0)) {
    ImGui::OpenPopup("Link Context Menu");
  } else if (ed::ShowBackgroundContextMenu()) {
    ImGui::OpenPopup("Create New Node");
  }
  ed::Resume();

  ed::Suspend();
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
  if (ImGui::BeginPopup("Node Context Menu")) {
    auto node = g.GetNode(select_node0.Get());
    ImGui::TextUnformatted("Node Context Menu");
    if (node) {
      ImGui::Text("Node ID: %ld", node->GetId());
      ImGui::Text("Node Name: %s", node->GetName().c_str());
      ImGui::Text("Node Type: %s", node->GetType().name());
      ImGui::Separator();
      if (ImGui::MenuItem("Delete?")) {
        ed::DeleteNode(select_node0);
      }
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("Link Context Menu")) {
    auto link = g.GetSocket(select_link0.Get());
    ImGui::TextUnformatted("Link Context Menu");
    if (link) {
      ImGui::Text("Link ID: %ld", link->id_);
      ImGui::Text("Link Input: %ld", link->input_->id_);
      ImGui::Text("Link Output: %ld", link->output_->id_);
      ImGui::Text("Link Type: %s", link->input_->descriptor_->type_.name());
      ImGui::Separator();
      if (ImGui::MenuItem("Delete?")) {
        ed::DeleteLink(select_link0);
      }
    }
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("Create New Node", ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted("Create New Node");
    ImGui::Separator();
    static char name_input[256]{0};
    static int current_size = 0;
    std::string front_name;
    if (ImGui::InputText("name", name_input, 256)) {
      current_size = strlen(name_input);
    }

    for (auto const& name : details::get_node_names()) {
      if (!start_with(name, name_input, current_size)) {
        continue;
      }
      if (front_name.empty()) {
        front_name = name;
      }
      if (ImGui::MenuItem(name.c_str())) {
        auto desc = details::get_node_descriptor(name);
        if (desc) {
          auto status = g.AddNode(desc);
          if (!status.ok()) {
            AX_LOG(ERROR) << status;
          }
        }
      }
    }
    // If pressed enter and there is only one item, then create it.
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) && front_name.size() > 0) {
      auto desc = details::get_node_descriptor(front_name);
      if (desc) {
        auto status = g.AddNode(desc);
        if (!status.ok()) {
          AX_LOG(ERROR) << status;
        }
      }
      name_input[0] = 0;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  ImGui::PopStyleVar();
  ed::Resume();
}

static void draw_once(gl::UiRenderEvent) {
  auto& g = ensure_resource<Graph>();
  static bool has_cycle = GraphExecutorBase(g).HasCycle();
  ImGui::SetNextWindowBgAlpha(0.7);
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::Begin(
      "Node editor", nullptr,
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoMove);

  static std::string message = "";
  static int beg = 0, end = 1;

  if (has_cycle) {
    ImGui::Text("Cycle detected!");
  } else {
    ImGui::Text("No cycle detected.");
  }
  ImGui::SameLine();
  if (ImGui::Button("Recheck Cycle")) {
    has_cycle = GraphExecutorBase(g).HasCycle();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    g.Clear();
  }
  ImGui::SameLine();
  if (ImGui::Button("Execute")) {
    auto executor = GraphExecutorBase(g);
    auto status = executor.Execute(beg, end);
    if (!status.ok()) {
      AX_LOG(ERROR) << status;
      message = status.message();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Export JSON")) {
    Serializer ser(g);
    auto json = ser.Serialize();
    std::cout << json << std::endl;
  }

  ImGui::Separator();

  ed::SetCurrentEditor(context_);
  ed::Begin("Node editor", ImVec2(0.0, 0.0f));
  g.ForeachNode(draw_node);
  g.ForeachSocket(draw_socket);
  handle_inputs();
  handle_selection();

  ed::End();
  ed::SetCurrentEditor(nullptr);

  ImGui::Separator();
  ImGui::TextUnformatted(message.c_str());
  ImGui::End();
}

static void init(gl::ContextInitEvent const&) {
  AX_LOG(INFO) << "Create editor context";
  context_ = ed::CreateEditor();
}

static void cleanup(gl::ContextDestroyEvent const&) {
  AX_LOG(INFO) << "Destroy editor context";
  ed::DestroyEditor(context_);
  context_ = nullptr;
}

void install_renderer(GraphRendererOptions opt) {
  opt_ = opt;
  connect<gl::ContextInitEvent, &init>();
  connect<gl::ContextDestroyEvent, &cleanup>();
  connect<gl::UiRenderEvent, &draw_once>();
}

using WidgetMap = absl::flat_hash_map<std::type_index, CustomNodeRender>;

void add_custem_node_render(std::type_index type, CustomNodeRender const& widget) {
  auto& map = ensure_resource<WidgetMap>();
  map.try_emplace(type, widget);
}

CustomNodeRender const* get_custom_node_render(std::type_index type) {
  auto& map = ensure_resource<WidgetMap>();
  auto it = map.find(type);
  if (it != map.end()) {
    return &(it->second);
  }
  return nullptr;
}

}  // namespace ax::graph
