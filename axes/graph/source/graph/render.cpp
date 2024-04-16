#include "ax/graph/render.hpp"

#include <imgui_canvas.h>
#include <imgui_node_editor.h>

#include <boost/json/stream_parser.hpp>
#include <filesystem>
#include <fstream>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/gl/context.hpp"
#include "ax/graph/executor.hpp"
#include "ax/graph/graph.hpp"
#include "ax/graph/node.hpp"
#include "ax/graph/serial.hpp"
#include "ax/utils/asset.hpp"
#include "ax/utils/status.hpp"

namespace ed = ax::NodeEditor;

inline bool match_char(char A, char B) { return A == B || std::tolower(A) == std::tolower(B); }

static bool partially_match(std::string const& str, const char* user_input, size_t length) {
  if (length > str.length()) {
    return false;
  }

  size_t j = 0, i = 0;
  while (i < length) {
    while (j < str.length()) {
      if (match_char(str[j], user_input[i])) {
        ++j;
        break;
      }
      ++j;
    }

    if (j == str.length()) {
      return i == length - 1;
    }
    ++i;
  }
  return i >= length;
}

namespace ax::graph {

///////////////////////////////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////////////////////////////

GraphRendererOptions opt_;
ed::EditorContext* context_;
ed::PinId hovered_pin_;
ed::NodeId hovered_node_;
std::string ax_blueprint_root;
bool is_open_;
bool has_cycle;
bool is_config_open_;
bool need_load_json;
bool need_export_json;
char json_out_path[64] = "blueprint.json";

UPtr<GraphExecutorBase> executor_;

void draw_node_header_default(NodeBase* node) {
  ImGui::TextColored(ImVec4(0.1, 0.5, 0.8, 1), "= %s", node->GetDescriptor()->name_.c_str());
}

void draw_node_content_default(NodeBase* node) {
  auto const& in = node->GetInputs();
  auto const& out = node->GetOutputs();
  size_t n_max_io = std::max(in.size(), out.size());
  absl::InlinedVector<real, 16> input_widths;
  float max_width = 0;
  for (size_t i = 0; i < in.size(); ++i) {
    auto size = ImGui::CalcTextSize(in[i].descriptor_->name_.c_str(), nullptr, true);
    input_widths.push_back(size.x);
    max_width = std::max(max_width, size.x);
  }
  for (size_t i = 0; i < n_max_io; ++i) {
    if (i < in.size()) {
      ed::BeginPin(in[i].id_, ed::PinKind::Input);
      ImGui::Text("%s", in[i].descriptor_->name_.c_str());
      ed::EndPin();
      ImGui::SameLine();
      ImGui::Dummy(ImVec2(max_width + 10 - input_widths[i], 0));
      ImGui::SameLine();
    } else {
      ImGui::Dummy(ImVec2(max_width + 10, 0));
      ImGui::SameLine();
    }
    if (i < out.size()) {
      ed::BeginPin(out[i].id_, ed::PinKind::Output);
      ImGui::TextUnformatted(out[i].descriptor_->name_.c_str());
      ed::EndPin();
    } else {
      ImGui::Dummy(ImVec2(0, 0));
    }
  }
}

void begin_draw_node(NodeBase* node) {
  ed::BeginNode(node->GetId());
  ImGui::PushID(node);
}

void end_draw_node() {
  ImGui::PopID();
  NodeEditor::EndNode();
}

void draw_node(NodeBase* node) {
  if (auto render = get_custom_node_render(node->GetType()); render) {
    render->widget_(node);
  } else {
    begin_draw_node(node);
    draw_node_header_default(node);
    draw_node_content_default(node);
    end_draw_node();
  }
}

void draw_socket(Socket* socket) {
  ed::Link(socket->id_, socket->input_->id_, socket->output_->id_);
}

void add_node(Graph& g, char const* name) {
  auto desc = details::get_node_descriptor(name);
  if (desc) {
    auto opt_node = g.AddNode(desc);
    if (!opt_node.ok()) {
      AX_LOG(ERROR) << opt_node;
    }
    auto node = opt_node.value();

    // current mouse position in the editor.
    draw_node(node);
    ed::CenterNodeOnScreen(node->GetId());
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Handling User Input
///////////////////////////////////////////////////////////////////////////////////////////////////
static void handle_inputs() {
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
          if (s != nullptr) {
            AX_LOG(INFO) << "Created link: " << s->input_->node_id_ << ":"
                         << s->input_->node_io_index_ << "->" << s->output_->node_id_ << ":"
                         << s->output_->node_io_index_;
            draw_socket(s);
          }
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
static void handle_selection() {
  auto& g = ensure_resource<Graph>();
  ed::NodeId nodes[1];
  ed::LinkId links[1];
  ed::GetSelectedNodes(nodes, 1);
  ed::GetSelectedLinks(links, 1);

  ed::NodeId select_node0 = nodes[0];
  ed::LinkId select_link0 = links[0];
  hovered_pin_ = ed::GetHoveredPin();
  hovered_node_ = ed::GetHoveredNode();

  ed::Suspend();
  if (ed::ShowNodeContextMenu(&select_node0)) {
    if (select_node0.Get() > 0) {
      ImGui::OpenPopup("Node Context Menu");
    }
  } else if (ed::ShowLinkContextMenu(&select_link0)) {
    if (select_link0.Get() > 0) {
      ImGui::OpenPopup("Link Context Menu");
    }
  } else if (ed::ShowBackgroundContextMenu()) {
    ImGui::OpenPopup("Create New Node");
  }

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
  if (ImGui::BeginPopup("Node Context Menu")) {
    auto node = g.GetNode(select_node0.Get());
    ImGui::TextUnformatted("Node Context Menu");
    if (node) {
      ImGui::Text("Node ID: %ld", node->GetId());
      ImGui::Text("Node Name: %s", node->GetDescriptor()->name_.c_str());
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
    const char* front_name = nullptr;
    if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::IsAnyItemActive()
        && !ImGui::IsMouseClicked(0))
      ImGui::SetKeyboardFocusHere();
    if (ImGui::InputText("name", name_input, 256)) {
      current_size = strlen(name_input);
    }

    for (auto const& name : details::get_node_names()) {
      if (!partially_match(name, name_input, current_size)) {
        continue;
      }
      if (front_name == nullptr) {
        front_name = name.c_str();
      }
      if (ImGui::MenuItem(name.c_str())) {
        add_node(g, name.c_str());
      }
    }
    // If pressed enter and there is only one item, then create it.
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) && front_name != nullptr) {
      add_node(g, front_name);
      name_input[0] = 0;
      current_size = 0;
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
  ed::Resume();
}

static void export_json(std::ostream& out) {
  auto& g = ensure_resource<Graph>();
  Serializer ser(g);
  g.ForeachNode([&](NodeBase* n) {
    ImVec2 pos = ed::GetNodePosition(n->GetId());
    boost::json::object meta = n->Serialize();
    meta["canvas_x"] = pos.x;
    meta["canvas_y"] = pos.y;
    ser.SetNodeMetadata(n->GetId(), meta);
  });
  auto json = ser.Serialize();
  out << json;
}

static void draw_hovered() {
  auto& g = ensure_resource<Graph>();
  if (hovered_pin_) {
    auto pin = g.GetPin(hovered_pin_.Get());
    if (ImGui::BeginTooltip()) {
      ImGui::TextUnformatted(pin->descriptor_->description_.c_str());
      ImGui::EndTooltip();
    }
    return;
  }

  if (hovered_node_) {
    auto node = g.GetNode(hovered_node_.Get());
    if (node) {
      if (ImGui::BeginTooltip()) {
        ImGui::TextUnformatted(node->GetDescriptor()->description_.c_str());
        ImGui::EndTooltip();
      }
    }
  }
}

static void draw_config_window(gl::UiRenderEvent) {
  if (!is_config_open_) {
    return;
  }
  if (!ImGui::Begin("Graph Commander", &is_config_open_,
       ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar)) {
    ImGui::End();
    return;
  }

  auto& g = ensure_resource<Graph>();
  has_cycle = GraphExecutorBase(g).HasCycle();

  static std::string message = "";
  static int end = 1;

  if (ImGui::Button("Open Editor")) {
    is_open_ = true;
  }
  ImGui::SameLine();

  if (ImGui::Button("Close Editor")) {
    is_open_ = false;
  }
  ImGui::SameLine();

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
    auto status = executor.Execute(end);
    if (!status.ok()) {
      AX_LOG(ERROR) << status;
      message = status.message();
    }
  }
  ImGui::SameLine();
  ImGui::SetNextItemWidth(100);
  ImGui::InputInt("End Frame", &end);


  need_export_json |= ImGui::Button("Export");
  ImGui::SameLine();
  need_load_json |= ImGui::Button("Load");
  ImGui::SameLine();
  ImGui::Text("Rel: %s", ax_blueprint_root.c_str());
  ImGui::InputText("Path", json_out_path, 64);
  ImGui::TextUnformatted(message.c_str());
  ImGui::End();
}

static void draw_once(gl::UiRenderEvent) {
  if (!is_open_) {
    return;
  }
  if (!ImGui::Begin("Node editor", &is_open_,
       ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar)) {
    ImGui::End();
    return;
  }

  auto& g = ensure_resource<Graph>();
  ed::SetCurrentEditor(context_);
  ed::Begin("Node editor", ImVec2(0.0, 0.0f));

  if (need_load_json) {
    std::ifstream file(ax_blueprint_root + json_out_path);
    if (!file) {
      AX_LOG(ERROR) << "Cannot open file: " << json_out_path;
    } else {
      Deserializer d(g);
      boost::json::stream_parser p;
      file.seekg(0, std::ios::end);
      size_t size = file.tellg();
      file.seekg(0, std::ios::beg);
      std::string buffer(size, ' ');
      file.read(buffer.data(), size);
      p.write(buffer);
      p.finish();
      auto json = p.release();
      auto s = d.Deserialize(json.as_object());
      if (!s.ok()) {
        AX_LOG(ERROR) << "Failed to load blue print!" << s;
      }
      has_cycle = GraphExecutorBase(g).HasCycle();

      auto const& meta = d.node_metadata_;
      auto const& node_id_map = d.inverse_node_id_map_;
      g.ForeachNode([&](NodeBase* n) {
        idx node_id_in_graph = n->GetId();
        idx node_id_in_json = node_id_map.at(node_id_in_graph);
        if (auto iter = meta.find(node_id_in_json); iter != meta.end()) {
          auto const& obj = iter->second.as_object();
          ImVec2 pos;
          pos.x = obj.at("canvas_x").as_double();
          pos.y = obj.at("canvas_y").as_double();
          ed::SetNodePosition(node_id_in_graph, pos);
          n->Deserialize(obj);
        }
      });
    }
    need_load_json = false;
  }

  g.ForeachNode(draw_node);
  g.ForeachSocket(draw_socket);
  handle_inputs();
  handle_selection();

  if (need_export_json) {
    std::ofstream file(ax_blueprint_root + json_out_path);
    if (!file) {
      AX_LOG(ERROR) << "Cannot open file: " << json_out_path;
    } else {
      export_json(file);
    }
    need_export_json = false;
  }

  // Handle other keyboard shortcuts.
  if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::IsAnyItemActive()
      && !ImGui::IsMouseClicked(0)) {
    if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
      if (!ImGui::IsWindowCollapsed()) {
        ImGui::OpenPopup("Create New Node");
      }
    }
  }
  draw_hovered();
  ed::End();
  ed::SetCurrentEditor(nullptr);
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

void on_menu_bar(gl::MainMenuBarRenderEvent) {
  if (ImGui::BeginMenu("Graph")) {
    if (ImGui::MenuItem("Open Graph Editor")) {
      is_open_ = true;
    }
    if (ImGui::MenuItem("Graph Commander")) {
      is_config_open_ = true;
    }
    ImGui::EndMenu();
  }
}

void install_renderer(GraphRendererOptions opt) {
  opt_ = opt;
  ax_blueprint_root = utils::get_root_dir();
  is_open_ = false;
  is_config_open_ = true;
  for (auto& c : ax_blueprint_root) {
    if (c == '\\') {
      c = '/';
    }
  }
  ax_blueprint_root = ax_blueprint_root + "/blueprints/";

  auto path = get_program_path();
  if (path) {
    std::string spath = path;
    for (auto& c : spath) {
      if (c == '\\') {
        c = '/';
      }
    }
    size_t last_slash = spath.find_last_of('/');
    if (last_slash != std::string::npos) {
      spath = spath.substr(last_slash + 1);
    }
    spath = spath + ".json";

    if (std::filesystem::exists(ax_blueprint_root + spath)) {
      std::ifstream file(ax_blueprint_root + spath);
      if (!file) {
        AX_LOG(ERROR) << "Cannot open file: " << spath;
      } else {
        need_load_json = true;
      }
    }

    for (size_t i = 0; i < spath.size(); ++i) {
      json_out_path[i] = spath[i];
    }
    json_out_path[spath.size()] = 0;
  }

  connect<gl::ContextInitEvent, &init>();
  connect<gl::ContextDestroyEvent, &cleanup>();
  connect<gl::UiRenderEvent, &draw_once>();
  connect<gl::UiRenderEvent, &draw_config_window>();
  connect<gl::MainMenuBarRenderEvent, &on_menu_bar>();
  add_clean_up_hook("Remove Graph", []() -> Status {
    erase_resource<Graph>();
    AX_RETURN_OK();
  });
}

using WidgetMap = absl::flat_hash_map<std::type_index, CustomNodeRender>;

void add_custom_node_render(std::type_index type, CustomNodeRender const& widget) {
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
