#include "ax/graph/render.hpp"

#include <imgui_canvas.h>
#include <imgui_node_editor.h>

#include <boost/json/stream_parser.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "ax/core/entt.hpp"
#include "ax/core/init.hpp"
#include "ax/core/logging.hpp"
#include "ax/gl/context.hpp"
#include "ax/graph/cache_sequence.hpp"
#include "ax/graph/executor.hpp"
#include "ax/graph/serial.hpp"
#include "ax/utils/asset.hpp"

namespace ed = ax::NodeEditor;

inline bool match_char(char A, char B) { return A == B || std::tolower(A) == std::tolower(B); }

static int partially_match(std::string const& str, const char* user_input, size_t length) {
  if (length > str.length()) {
    return false;
  }

  size_t j = 0, i = 0;
  for (; i < str.length() && j < length; ++i) {
    if (match_char(str[i], user_input[j])) {
      ++j;
    }
  }
  return j == length;
}

namespace ax::graph {

///////////////////////////////////////////////////////////////////////////////////////////////////
// Global Data
///////////////////////////////////////////////////////////////////////////////////////////////////

GraphRendererOptions opt_;
ed::EditorContext* context_;
ed::PinId hovered_pin_ = ed::PinId::Invalid;
ed::NodeId hovered_node_ = ed::NodeId::Invalid;
std::string ax_blueprint_root;
bool is_open_;
bool has_cycle;
bool is_config_open_;
bool need_load_json = false;
bool need_export_json;
bool running_;
char json_out_path[64] = "blueprint.json";

std::unique_ptr<GraphExecutorBase>& ensure_executor() {
  auto& g = ensure_resource<Graph>();
  if (auto* ptr = try_get_resource<std::unique_ptr<GraphExecutorBase>>()) {
    if_likely(&(ptr->get()->GetGraph()) == &g) { return *ptr; }
    else {
      erase_resource<std::unique_ptr<GraphExecutorBase>>();
      return add_resource<std::unique_ptr<GraphExecutorBase>>(std::make_unique<GraphExecutorBase>(g));
    }
  } else {
    auto& r = add_resource<std::unique_ptr<GraphExecutorBase>>(std::make_unique<GraphExecutorBase>(g));
    return r;
  }
}

void draw_node_header_default(NodePtr node) {
  ImGui::TextColored(ImVec4(0.1, 0.5, 0.8, 1), "= %s", node->GetDescriptor().Name().c_str());
}

void draw_node_content_default(NodeBase* node) {
  auto const& in = node->GetInputs();
  auto const& out = node->GetOutputs();
  size_t n_max_io = std::max(in.size(), out.size());
  std::vector<float> input_widths;
  std::vector<float> output_widths;
  float max_input_width = 0;
  const auto& input_sockets = node->GetDescriptor().GetInputs();
  const auto& output_sockets = node->GetDescriptor().GetOutputs();

  for (const auto& in_sock : node->GetDescriptor().GetInputs()) {
    const char* name = in_sock.Name().c_str();
    ImVec2 size = ImGui::CalcTextSize(name);
    max_input_width = std::max(max_input_width, size.x);
  }

  for (const auto& out_sock : node->GetDescriptor().GetOutputs()) {
    const char* name = out_sock.Name().c_str();
    ImVec2 size = ImGui::CalcTextSize(name);
    output_widths.push_back(size.x);
  }

  for (size_t i = 0; i < n_max_io; ++i) {
    if (i < in.size()) {
      ed::PinId id{&in[i]};
      ed::BeginPin(id, ed::PinKind::Input);
      ImGui::TextUnformatted(input_sockets[i].Name().c_str());
      ed::EndPin();
      // ImGui::SameLine();
      // ImGui::Dummy(ImVec2(max_input_width + 10 - input_widths[i], 0));
      // ImGui::SameLine();
    } else {
      // ImGui::Dummy(ImVec2(max_input_width + 10, 0));
      // ImGui::SameLine();
    }
    if (i < out.size()) {
      ed::PinId id{&out[i]};
      ed::BeginPin(id, ed::PinKind::Output);
      ImGui::TextUnformatted(output_sockets[i].Name().c_str());
      ed::EndPin();
    } else {
      // ImGui::Dummy(ImVec2(0, 0));
    }
  }
}

void begin_draw_node(NodeBase* node) {
  ed::NodeId id{node};
  ed::BeginNode(id);
  ImGui::PushID(id.AsPointer());
}

void end_draw_node() {
  NodeEditor::EndNode();
  ImGui::PopID();
}

void draw_node(NodeBase* node) {
  if (auto render = get_custom_node_render(node->GetDescriptor().Name()); render) {
    render->widget_(node);
  } else {
    begin_draw_node(node);
    draw_node_header_default(node);
    draw_node_content_default(node);
    end_draw_node();
  }
}

static void draw_link(Link<> const& link) {
  ed::PinId in_sock{link.From().operator->()};
  ed::PinId out_sock{link.To().operator->()};

  ed::LinkId id{link.To().operator->()};
  ed::Link(id, in_sock, out_sock);
}

static void add_node(Graph& g, char const* name) {
  auto const& reg = NodeRegistry::instance();
  auto node_ptr = reg.Create(name);  // May throw?
  g.PushBack(std::move(node_ptr));
}

static void do_connect_sockets(Graph& g, OutputSocket const* out, InputSocket const* in) try {
  auto out_nh = g.Get(&out->Node());
  auto in_nh = g.Get(&in->Node());
  AX_DCHECK(out_nh, "Output node handle not found.");
  AX_DCHECK(in_nh, "Input node handle not found.");
  auto out_sock_handle = out_nh->GetOutput(out->Index());
  auto in_sock_handle = in_nh->GetInput(in->Index());
  auto link = g.Connect(out_sock_handle, in_sock_handle);
  draw_link(link);
} catch (std::exception const& e) {
  AX_ERROR("Cannot create link: {}", e.what());
}

static void do_erase_link(Graph& g, InputSocket* in) {
  auto in_nh = g.Get(&in->Node());
  auto in_sock_handle = in_nh->GetInput(in->Index());
  AX_CHECK(in_sock_handle->IsConnected(), "Socket is not connected.");
  OutputSocket const* out = in_sock_handle->From();
  auto out_nh = g.Get(&out->Node());
  auto out_sock_handle = out_nh->GetOutput(out->Index());
  auto link = g.GetConnect(out_sock_handle, in_sock_handle);
  g.Erase(link.value());
}

static void do_erase_node(Graph& g, NodeBase* node) {
  auto nh = g.Get(node);
  g.Erase(nh.value());
}

static void query_then_erase(Graph& g) {
  ed::LinkId link_id = 0;
  while (ed::QueryDeletedLink(&link_id)) {
    if (ed::AcceptDeletedItem()) {
      AX_INFO("delete link: {}", link_id.Get());
      do_erase_link(g, link_id.AsPointer<InputSocket>());
    }
  }

  ed::NodeId node_id = 0;
  while (ed::QueryDeletedNode(&node_id)) {
    if (ed::AcceptDeletedItem()) {
      AX_INFO("delete node: {}", node_id.Get());
      do_erase_node(g, node_id.AsPointer<NodeBase>());
    }
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
// Handling User Input
///////////////////////////////////////////////////////////////////////////////////////////////////
static void handle_inputs() {
  auto& g = ensure_resource<Graph>();
  if (ed::BeginCreate()) {
    ed::PinId input_id, output_id;  // NOTE: I/O of Link is O/I of each node!
    if (ed::QueryNewLink(&input_id, &output_id)) {
      if (input_id && output_id) {
        if (ed::AcceptNewItem()) {
          SocketBase const* out = input_id.AsPointer<SocketBase>();
          SocketBase const* in = output_id.AsPointer<SocketBase>();
          if (!out->IsInput() && in->IsInput()) {
            do_connect_sockets(g, static_cast<OutputSocket const*>(out), static_cast<InputSocket const*>(in));
          } else {
            do_connect_sockets(g, static_cast<OutputSocket const*>(in), static_cast<InputSocket const*>(out));
          }
        }
      }
    }
  }
  ed::EndCreate();

  if (ed::BeginDelete()) {
    query_then_erase(g);
  }
  ed::EndDelete();
}

static void show_create_node_window(Graph& g) {
  ImGui::TextUnformatted("Create New Node");
  ImGui::Separator();
  static char name_input[256]{0};
  static size_t current_size = 0;
  const char* front_name = nullptr;

  if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::IsAnyItemActive()
      && !ImGui::IsMouseClicked(0))
    ImGui::SetKeyboardFocusHere();

  if (ImGui::InputText("name", name_input, 256)) {
    current_size = strlen(name_input);
  }

  auto const& desc = NodeRegistry::instance().GetDescriptors();

  for (auto const& [name, _] : desc) {
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
}

// TODO: static void show_link_context_menu(ed::LinkId select_link0, Socket* link) {
//   ImGui::TextUnformatted("Link Context Menu");
//   if (link) {
//     ImGui::Text("Link ID: %ld", link->id_);
//     ImGui::Text("Link Input: %ld", link->input_->id_);
//     ImGui::Text("Link Output: %ld", link->output_->id_);
//     ImGui::Text("Link Type: %s", link->input_->descriptor_->type_.name());
//     ImGui::Separator();
//     if (ImGui::MenuItem("Delete?")) {
//       ed::DeleteLink(select_link0);
//     }
//   }
// }

static void show_node_context_menu(ed::NodeId select_node0, NodeBase const* node) {
  ImGui::TextUnformatted("Node Context Menu");
  if (node) {
    ImGui::Text("Node Name: %s", node->GetDescriptor().Name().c_str());
    ImGui::Text("Node Type: %s", node->GetDescriptor().Desc().c_str());
    ImGui::Separator();
    if (ImGui::MenuItem("Delete?")) {
      ed::DeleteNode(select_node0);
    }
  }
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
    auto* ptr = select_node0.AsPointer<NodeBase>();
    show_node_context_menu(select_node0, ptr);
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("Create New Node", ImGuiWindowFlags_AlwaysAutoResize)) {
    show_create_node_window(g);
    ImGui::EndPopup();
  }
  ed::Resume();
}

static void serialize_graph(std::ostream& out) {
  auto& g = ensure_resource<Graph>();
  Serializer ser(g);
  for (auto const& node : g.GetNodes()) {
    NodeBase const* ptr = node.get();
    ImVec2 const pos = ed::GetNodePosition(ed::NodeId(ptr));
    boost::json::object meta = node->Serialize();
    meta["canvas_x"] = pos.x;
    meta["canvas_y"] = pos.y;
    ser.SetNodeMetadata(ptr, meta);
  }

  try {
    auto json = ser.Serialize();
    out << json;
  } catch (std::exception const& e) {
    AX_CRITICAL("Failed export graph to json file: {}", e.what());
  }
}

static void draw_hovered() {
  auto& g = ensure_resource<Graph>();
  if (hovered_pin_) {
    // TODO: impl?
    //
    // if (auto const* pin = g.GetPin(hovered_pin_.Get())) {
    //   if (ImGui::BeginTooltip()) {
    //     ImGui::TextUnformatted(pin->descriptor_->description_.c_str());
    //     ImGui::EndTooltip();
    //   }
    // }
    // return;
  }

  if (hovered_node_) {
    // TODO: impl?
    //
    // if (auto const* node = g.GetNode(hovered_node_.Get())) {
    //   if (ImGui::BeginTooltip()) {
    //     ImGui::TextUnformatted(node->GetDescriptor()->description_.c_str());
    //     ImGui::EndTooltip();
    //   }
    // }
  }
}

static void draw_config_window(gl::UiRenderEvent) {
  if (!is_config_open_) {
    return;
  }
  if (!ImGui::Begin("Graph Commander", &is_config_open_,
                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::End();
    return;
  }

  auto& g = ensure_resource<Graph>();
  has_cycle = ensure_executor()->HasCycle();

  static std::string message;
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
    has_cycle = ensure_executor()->HasCycle();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    g.Clear();
  }
  ImGui::Separator();

  auto& executor = ensure_executor();

  ImGui::SetNextItemWidth(100);
  if (ImGui::InputInt("End Frame", &end)) {
    if (end < 0) {
      end = 0;
    }
    executor->SetEnd(end);
  }
  ImGui::SameLine();
  if (ImGui::Button("Execute All")) {
    // TODO: Try except
    executor->Execute();
  }
  ImGui::SameLine();
  if (ImGui::Button("Execute Once")) {
    // run_once();
    AX_ERROR("Not Implemented");
  }
  ImGui::Checkbox("Running", &running_);
  ImGui::SameLine();
  ImGui::Text("Current Frame: %ld", executor->GetCurrentFrame());
  ImGui::Separator();

  /* Cache Sequence */
  static int cache_in_show = 0;
  ImGui::SetNextItemWidth(150);
  bool need_trigger_event = ImGui::InputInt("Cache Sequence", &cache_in_show, 1, 0, end);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(200);
  need_trigger_event |= ImGui::DragInt("Idx", &cache_in_show, 0, end);

  /* Import/Export */
  ImGui::Separator();
  need_export_json |= ImGui::Button("Export");
  ImGui::SameLine();
  need_load_json |= ImGui::Button("Load");
  ImGui::SameLine();
  ImGui::Text("Rel: %s", ax_blueprint_root.c_str());
  ImGui::InputText("Path", json_out_path, 64);
  ImGui::TextUnformatted(message.c_str());
  ImGui::End();

  if (need_trigger_event) {
    CacheSequenceUpdateEvent e;
    e.required_frame_id_ = cache_in_show;
    e.is_cleanup_ = false;
    emit_enqueue(e);
    trigger_queue();
  }
}

static void deserialize_graph(Graph& g, std::ifstream& file) {
  boost::json::stream_parser p;
  file.seekg(0, std::ios::end);
  auto size = file.tellg();
  file.seekg(0, std::ios::beg);
  if (size < 0) {
    AX_CRITICAL("File load error.");
    return;
  }

  std::string buffer(static_cast<size_t>(size), ' ');
  file.read(buffer.data(), size);
  if (!file.good()) {
    AX_CRITICAL("File load error.");
    return;
  }
  p.write(buffer);
  p.finish();

  if (!p.done()) {
    AX_CRITICAL("Failed to deserialize: json invalid");
    return;
  }
  boost::json::value json = p.release();

  Serializer d(g);
  try {
    d.Deserialize(json.as_object());
  } catch (std::exception const& e) {
    AX_CRITICAL("Failed to deserialize: {}", e.what());
    g.Clear();
    return;
  }

  has_cycle = ensure_executor()->HasCycle();

  for (auto const& [ptr, obj] : d.GetNodeMetadata()) {
    ImVec2 pos;
    auto nh = g.Get(ptr);
    if (!nh) {
      continue;
    }
    pos.x = static_cast<float>(obj.at("canvas_x").as_double());
    pos.y = static_cast<float>(obj.at("canvas_y").as_double());
    ed::SetNodePosition(ed::NodeId(ptr), pos);
    (*nh)->Deserialize(obj);
  }
}

static void draw_once(gl::UiRenderEvent) {
  if (!is_open_) {
    return;
  }
  if (!ImGui::Begin("Node editor", &is_open_, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar)) {
    ImGui::End();
    return;
  }

  auto& g = ensure_resource<Graph>();
  ed::SetCurrentEditor(context_);
  ed::Begin("Node editor", ImVec2(0.0, 0.0f));

  if (need_load_json) {
    std::ifstream file(ax_blueprint_root + json_out_path);
    if (!file) {
      AX_ERROR("Cannot open file: {}", json_out_path);
    } else {
      deserialize_graph(g, file);
    }
    need_load_json = false;
  }

  // g.ForeachNode(draw_node);
  for (auto const& n : g.GetNodes()) {
    draw_node(n.get());

    auto handle = g.Get(n.get());
    for (auto const& in_sock : n->GetInputs()) {
      auto in_handle = handle->GetInput(in_sock.Index());
      if (in_handle->IsConnected()) {
        const auto* out_sock = in_handle->From();
        auto out_nh = g.Get(&out_sock->Node());
        AX_CHECK(out_nh, "Cannot find node.");
        auto out_handle = out_nh->GetOutput(out_sock->Index());
        auto link = g.GetConnect(out_handle, in_handle);
        AX_CHECK(link, "Cannot find link.");
        draw_link(*link);
      }
    }
  }
  handle_inputs();
  handle_selection();

  if (need_export_json) {
    std::ofstream file(ax_blueprint_root + json_out_path);
    if (!file) {
      AX_ERROR("Cannot open file: {}", json_out_path);
    } else {
      serialize_graph(file);
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
  AX_TRACE("Create editor context");
  context_ = ed::CreateEditor();
}

static void cleanup(gl::ContextDestroyEvent const&) {
  AX_TRACE("Destroy editor context");
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

void on_tick_logic(gl::TickLogicEvent) {
  if (!running_) {
    return;
  }
  // run_once();
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

  const char* path = get_program_path();
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
    if (spath.ends_with(".exe")) {
      spath = spath.substr(0, spath.size() - 4);
    }
    spath = spath + ".json";

    if (std::filesystem::exists(ax_blueprint_root + spath)) {
      std::ifstream file(ax_blueprint_root + spath);
      if (!file) {
        AX_ERROR("Cannot open file: {}", spath);
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
  connect<gl::TickLogicEvent, &on_tick_logic>();

  add_clean_up_hook("Remove Graph", []() { erase_resource<Graph>(); });
}

using WidgetMap = std::unordered_map<std::string, CustomNodeRender>;

void add_custom_node_render(std::string type, CustomNodeRender const& widget) {
  auto& map = ensure_resource<WidgetMap>();
  map.try_emplace(type, widget);
}

CustomNodeRender const* get_custom_node_render(std::string name) {
  auto& map = ensure_resource<WidgetMap>();
  auto it = map.find(name);
  if (it != map.end()) {
    return &(it->second);
  }
  return nullptr;
}

}  // namespace ax::graph
