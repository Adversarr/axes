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
#include "ax/graph/graph.hpp"
#include "ax/graph/node.hpp"
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
ed::PinId hovered_pin_;
ed::NodeId hovered_node_;
std::string ax_blueprint_root;
bool is_open_;
bool has_cycle;
bool is_config_open_;
bool need_load_json;
bool need_export_json;
bool running_;
char json_out_path[64] = "blueprint.json";

static void run_once();

std::unique_ptr<GraphExecutorBase>& ensure_executor() {
  auto& g = ensure_resource<Graph>();
  if (auto* ptr = try_get_resource<std::unique_ptr<GraphExecutorBase>>()) {
    if_likely(&(ptr->get()->GetGraph()) == &g) { return *ptr; }
    else {
      erase_resource<std::unique_ptr<GraphExecutorBase>>();
      return add_resource<std::unique_ptr<GraphExecutorBase>>(
          std::make_unique<GraphExecutorBase>(g));
    }
  } else {
    auto& r
        = add_resource<std::unique_ptr<GraphExecutorBase>>(std::make_unique<GraphExecutorBase>(g));
    return r;
  }
}

void draw_node_header_default(NodeBase* node) {
  ImGui::TextColored(ImVec4(0.1, 0.5, 0.8, 1), "= %s", node->GetDescriptor()->name_.c_str());
}

void draw_node_content_default(NodeBase* node) {
  auto const& in = node->GetInputs();
  auto const& out = node->GetOutputs();
  size_t n_max_io = std::max(in.size(), out.size());
  std::vector<float> input_widths;
  float max_width = 0;
  for (const auto& i : in) {
    auto size = ImGui::CalcTextSize(i.descriptor_->name_.c_str(), nullptr, true);
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

static void draw_socket(Socket* socket) {
  ed::Link(socket->id_, socket->input_->id_, socket->output_->id_);
}

static void add_node(Graph& g, char const* name) {
  auto desc = details::get_node_descriptor(name);
  if (desc) {
    // TODO: Try except here
    try {
      auto* node = g.AddNode(desc);
      // current mouse position in the editor.
      draw_node(node);
      ed::CenterNodeOnScreen(node->GetId());
    } catch (std::exception const& e) {
      AX_ERROR("Cannot create node: {}", e.what());
    }
  }
}

static void do_create(Graph& g, size_t from_id, size_t to_id) {
  auto* from = g.GetPin(from_id);
  auto* to = g.GetPin(to_id);
  if (from == nullptr) {
    AX_ERROR("Cannot find from-pin: {}", from_id);
    return;
  } else if (to == nullptr) {
    AX_ERROR("Cannot find to-pin: {}", to_id);
    return;
  }

  // determine from/to to in/out
  Pin *input = nullptr, *output = nullptr;
  if (from->is_input_ && !to->is_input_) {
    input = to;
    output = from;
  } else if (!from->is_input_ && to->is_input_) {
    input = from;
    output = to;
  } else {
    AX_ERROR("Cannot connect two input or two output pins.");
    return;
  }

  // TODO: Try except
  Socket* sock = nullptr;
  try {
    sock = g.AddSocket(input, output);
  } catch (std::exception const& e) {
    AX_ERROR("Cannot create link: {}", e.what());
  }

  if (sock != nullptr) {
    AX_INFO("Create link: {}:{} -> {}:{}", sock->input_->node_id_, sock->input_->node_io_index_,
            sock->output_->node_id_, sock->output_->node_io_index_);
    draw_socket(sock);
  }
}

static void do_delete(Graph& g) {
  ed::LinkId link_id = 0;
  while (ed::QueryDeletedLink(&link_id)) {
    if (ed::AcceptDeletedItem()) {
      AX_INFO("Deleted link: {}", link_id.Get());
      g.RemoveSocket(link_id.Get());
    }
  }

  ed::NodeId node_id = 0;
  while (ed::QueryDeletedNode(&node_id)) {
    if (ed::AcceptDeletedItem()) {
      AX_INFO("Deleted node: {}", node_id.Get());
      g.RemoveNode(node_id.Get());
    }
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
        do_create(g, input_id.Get(), output_id.Get());
      }
    }
  }
  ed::EndCreate();
  if (ed::BeginDelete()) {
    do_delete(g);
  }
  ed::EndDelete();
}

static void show_create_node_window(Graph& g) {
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
}
static void show_link_context_menu(ed::LinkId select_link0, Socket* link) {
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
}
static void show_node_context_menu(ed::NodeId select_node0, NodeBase* node) {
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
    auto* node = g.GetNode(select_node0.Get());
    show_node_context_menu(select_node0, node);
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("Link Context Menu")) {
    auto* link = g.GetSocket(select_link0.Get());
    show_link_context_menu(select_link0, link);
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
  try {
    Serializer ser(g);
    g.ForeachNode([&](NodeBase* n) {
      ImVec2 const pos = ed::GetNodePosition(n->GetId());
      boost::json::object meta = n->Serialize();
      meta["canvas_x"] = pos.x;
      meta["canvas_y"] = pos.y;
      ser.SetNodeMetadata(n->GetId(), meta);
    });
    auto json = ser.Serialize();
    out << json;
  } catch (std::exception const& e) {
    AX_CRITICAL("Failed export graph to json file: {}", e.what());
  }
}

static void draw_hovered() {
  auto& g = ensure_resource<Graph>();
  if (hovered_pin_) {
    if (auto const* pin = g.GetPin(hovered_pin_.Get())) {
      if (ImGui::BeginTooltip()) {
        ImGui::TextUnformatted(pin->descriptor_->description_.c_str());
        ImGui::EndTooltip();
      }
    }
    return;
  }

  if (hovered_node_) {
    if (auto const* node = g.GetNode(hovered_node_.Get())) {
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
                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar
                        | ImGuiWindowFlags_AlwaysAutoResize)) {
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
    run_once();
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
  size_t size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::string buffer(size, ' ');
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

  Deserializer d(g);
  try {
    d.Deserialize(json.as_object());
  } catch (std::exception const& e) {
    AX_CRITICAL("Failed to deserialize: {}", e.what());
    g.Clear();
    return;
  }

  has_cycle = ensure_executor()->HasCycle();
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
      AX_ERROR("Cannot open file: {}", json_out_path);
    } else {
      deserialize_graph(g, file);
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

static void run_once() {
  auto& e = ensure_executor();
  auto stage = e->GetStage();
  switch (stage) {
    case GraphExecuteStage::kIdle: {
      // TODO: Try except
      e->Begin();
    }
    case GraphExecuteStage::kPrePreApply:
    case GraphExecuteStage::kPreApplyDone:
    case GraphExecuteStage::kApplyRunning:
    case GraphExecuteStage::kApplyDone:
    case GraphExecuteStage::kPrePostApply:
    case GraphExecuteStage::kPostApplyDone: {
      // TODO: Try except
      e->WorkOnce();
      break;
    }

    case GraphExecuteStage::kPreApplyError:
    case GraphExecuteStage::kApplyError:
    case GraphExecuteStage::kPostApplyError: {
      AX_ERROR("Error in executor: {}", int(stage));
      std::cout << "Error.." << std::endl;
      running_ = false;
      break;
    }
    case GraphExecuteStage::kDone: {
      e->End();
      running_ = false;
    }
  }
}

void on_tick_logic(gl::TickLogicEvent) {
  if (!running_) {
    return;
  }
  run_once();
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

using WidgetMap = std::unordered_map<std::type_index, CustomNodeRender>;

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
