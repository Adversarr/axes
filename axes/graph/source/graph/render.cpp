#include "ax/graph/render.hpp"

#include <imgui_canvas.h>
#include <imgui_node_editor.h>

#include <boost/json/stream_parser.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <range/v3/view/filter.hpp>

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
ed::EditorContext* context_ = nullptr;
ed::PinId hovered_pin_ = ed::PinId::Invalid;
ed::NodeId hovered_node_ = ed::NodeId::Invalid;
ed::LinkId hovered_link_ = ed::LinkId::Invalid;
ed::NodeId selected_node_ = ed::NodeId::Invalid;
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
  if (auto* ptr = try_get_resource<std::unique_ptr<GraphExecutorBase> >()) {
    if_likely(&(ptr->get()->GetGraph()) == &g) { return *ptr; } else {
      erase_resource<std::unique_ptr<GraphExecutorBase> >();
      return add_resource<std::unique_ptr<GraphExecutorBase> >(
          std::make_unique<GraphExecutorBase>(g));
    }
  } else {
    auto& r
        = add_resource<std::unique_ptr<GraphExecutorBase> >(std::make_unique<GraphExecutorBase>(g));
    return r;
  }
}

void draw_node_header_default(NodePtr node) {
  ImGui::TextColored(ImVec4(0.1f, 0.5f, 0.8f, 1), "= %s", node->GetDescriptor().Name().c_str());
}

void draw_node_content_default(NodeBase* node) {
  auto const& in = node->GetInputs();
  auto const& out = node->GetOutputs();
  size_t n_max_io = std::max(in.size(), out.size());
  float max_input_width = 0;
  float max_output_width = 0;
  const auto& input_sockets = node->GetDescriptor().GetInputs();
  const auto& output_sockets = node->GetDescriptor().GetOutputs();

  float title_width = ImGui::CalcTextSize(node->GetDescriptor().Name().c_str()).x;

  for (const auto& in_sock : node->GetDescriptor().GetInputs()) {
    const char* name = in_sock.Name().c_str();
    ImVec2 size = ImGui::CalcTextSize(name);
    max_input_width = std::max(max_input_width, size.x);
  }

  for (const auto& out_sock : node->GetDescriptor().GetOutputs()) {
    const char* name = out_sock.Name().c_str();
    ImVec2 size = ImGui::CalcTextSize(name);
    max_output_width = std::max(max_output_width, size.x);
  }

  float width = std::max(max_input_width + max_output_width, title_width) + 20;

  for (size_t i = 0; i < n_max_io; ++i) {
    auto line_start_cursor = ImGui::GetCursorScreenPos();
    if (i < in.size()) {
      ed::PinId id{&in[i]};
      ed::BeginPin(id, ed::PinKind::Input);
      ImGui::TextUnformatted(input_sockets[i].Name().c_str());
      ed::EndPin();
    } else {
      ImGui::Dummy({0, 0});
    }

    if (i < out.size()) {
      ImGui::SameLine();
      ed::PinId id{&out[i]};
      float current_size = ImGui::CalcTextSize(output_sockets[i].Name().c_str()).x;
      float required_cursor = line_start_cursor.x + width - current_size;

      ImGui::SetCursorScreenPos({required_cursor, line_start_cursor.y});
      ed::BeginPin(id, ed::PinKind::Output);
      ImGui::TextUnformatted(output_sockets[i].Name().c_str());
      ed::EndPin();
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

static void draw_node(NodeBase* node) {
  if (const auto* render = get_custom_node_render(node->GetDescriptor().Name()); render) {
    render->widget_(node);
  } else {
    begin_draw_node(node);
    draw_node_header_default(node);
    draw_node_content_default(node);
    end_draw_node();
  }
}

static void draw_link(Link<> const& link) {
  ed::PinId out_sock{link.From().operator->()};
  ed::PinId in_sock{link.To().operator->()};
  ed::LinkId id{link.To().operator->()};
  if (!ed::Link(id, in_sock, out_sock)) {
    AX_ERROR("Failed to draw link: {}, {}", fmt::ptr(in_sock.AsPointer<>()),
             fmt::ptr(out_sock.AsPointer<>()));
  }
}

static NodeBase* add_node(Graph& g, char const* name) {
  auto& reg = get_internal_node_registry();
  reg.LoadDefered();
  auto node_ptr = reg.Create(name); // May throw?
  auto nh = g.PushBack(std::move(node_ptr));
  draw_node(nh.operator->());
  return nh.operator->();
}

static bool do_connect_sockets(Graph& g, OutputSocket const* out, InputSocket const* in) try {
  auto out_nh = g.Get(&out->Node());
  auto in_nh = g.Get(&in->Node());
  AX_CHECK(out_nh, "Output node handle not found.");
  AX_CHECK(in_nh, "Input node handle not found.");
  auto out_sock_handle = out_nh->GetOutput(out->Index());
  auto in_sock_handle = in_nh->GetInput(in->Index());
  g.Connect(out_sock_handle, in_sock_handle);
  AX_INFO("Connected: {} -> {}", fmt::ptr(out), fmt::ptr(in));
  return true;
} catch (std::exception const& e) {
  AX_ERROR("Cannot create link: {}", e.what());
  return false;
}

static bool do_erase_link(Graph& g, InputSocket* in) {
  auto in_nh = g.Get(&in->Node());
  if (!in_nh) {
    AX_ERROR("Node not found.");
    return false;
  }
  auto in_sock_handle = in_nh->GetInput(in->Index());
  if (!in_sock_handle->IsConnected()) {
    AX_ERROR("Socket is not connected.");
    return false;
  }
  OutputSocket const* out = in_sock_handle->From();
  auto out_nh = g.Get(&out->Node());
  auto out_sock_handle = out_nh->GetOutput(out->Index());
  auto link = g.GetConnect(out_sock_handle, in_sock_handle);
  g.Erase(link.value());
  return true;
}

static void do_erase_node(Graph& g, NodeBase* node) {
  auto nh = g.Get(node);
  g.Erase(nh.value());
}

static void query_then_erase(Graph& g) {
  ed::LinkId link_id = ed::LinkId::Invalid;
  while (ed::QueryDeletedLink(&link_id)) {
    if (ed::AcceptDeletedItem()) {
      AX_INFO("delete link: {}", fmt::ptr(link_id.AsPointer<>()));
      do_erase_link(g, link_id.AsPointer<InputSocket>());
    }
  }

  ed::NodeId node_id = 0;
  while (ed::QueryDeletedNode(&node_id)) {
    if (ed::AcceptDeletedItem()) {
      AX_INFO("delete node: {}", fmt::ptr(node_id.AsPointer<>()));
      do_erase_node(g, node_id.AsPointer<NodeBase>());
    }
  }
}

static bool try_do_connect_pin(Graph& g, ed::PinId input_id, ed::PinId output_id) {
  SocketBase const* out = input_id.AsPointer<SocketBase>();
  SocketBase const* in = output_id.AsPointer<SocketBase>();
  if (!out->IsInput() && in->IsInput()) {
    return do_connect_sockets(g, static_cast<OutputSocket const*>(out),
                              static_cast<InputSocket const*>(in));
  } else if (out->IsInput() && !in->IsInput()) {
    return do_connect_sockets(g, static_cast<OutputSocket const*>(in),
                              static_cast<InputSocket const*>(out));
  } else if (out->IsInput() && in->IsInput()) {
    AX_ERROR("Cannot connect two input sockets.");
    return false;
  } else {
    AX_ERROR("Cannot connect two output sockets.");
    return false;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Handling User Input
///////////////////////////////////////////////////////////////////////////////////////////////////
static void handle_inputs() {
  auto& g = ensure_resource<Graph>();
  if (ed::BeginCreate()) {
    ed::PinId input_id, output_id; // NOTE: I/O of Link is O/I of each node!
    if (ed::QueryNewLink(&input_id, &output_id)) {
      if (input_id && output_id) {
        if (ed::AcceptNewItem()) {
          try_do_connect_pin(g, input_id, output_id);
        }
      }
    }
  }
  ed::EndCreate();

  if (ed::BeginDelete()) {
    query_then_erase(g);
  }
  ed::EndDelete();

  // Handle other keyboard shortcuts.
  if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::IsAnyItemActive()
      && !ImGui::IsMouseClicked(0)) {
    if (ImGui::IsKeyPressed(ImGuiKey_Space)) {
      if (!ImGui::IsWindowCollapsed()) {
        ImGui::OpenPopup("Create New Node");
      }
    }
  }
}

static void show_create_node_window(Graph& g) {
  ImGui::TextUnformatted("Create New Node");
  ImGui::Separator();
  static char name_input[256]{};
  static ImVec2 position;
  position = ImGui::GetMousePos();

  static size_t current_size = 0;
  const char* front_name = nullptr;

  if (ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) && !ImGui::IsAnyItemActive()
      && !ImGui::IsMouseClicked(0)) {
    ImGui::SetKeyboardFocusHere();
  }

  if (ImGui::InputText("Name", name_input, 256)) {
    current_size = strlen(name_input);
  }

  auto const& desc = get_internal_node_registry().GetDescriptors();

  NodeBase* ptr = nullptr;
  for (auto const& [name, _] : desc) {
    if (!partially_match(name, name_input, current_size)) {
      continue;
    }
    if (front_name == nullptr) {
      front_name = name.c_str();
    }
    if (ImGui::MenuItem(name.c_str())) {
      ptr = add_node(g, name.c_str());
    }
  }

  // If pressed enter and there is only one item, then create it.
  if (ImGui::IsKeyPressed(ImGuiKey_Enter)) {
    if (front_name != nullptr) {
      ptr = add_node(g, front_name);
      name_input[0] = 0;
      current_size = 0;
    } else {
      AX_ERROR("Cannot find node: {}", name_input);
    }
    ImGui::CloseCurrentPopup();
  }

  if (ptr) {
    ed::SetNodePosition(ed::NodeId(ptr), ed::ScreenToCanvas(position));
  }
}

static void show_link_context_menu(ed::LinkId select_link0, InputSocket* link) {
  ImGui::TextUnformatted("Link Context Menu");
  if (link) {
    ImGui::Text("InputSocket addr: %p", link);
    ImGui::Text("OutputSocket addr: %p", link->From());
    ImGui::Separator();
    if (ImGui::MenuItem("Delete?")) {
      ed::DeleteLink(select_link0);
    }
  }
}

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
  ed::LinkId links[1];
  ed::NodeId nodes[1];
  nodes[0] = ed::NodeId::Invalid;
  ed::GetSelectedNodes(nodes, 1);
  ed::GetSelectedLinks(links, 1);
  selected_node_ = nodes[0];

  ed::NodeId select_node0 = nodes[0];
  ed::LinkId select_link0 = links[0];
  hovered_pin_ = ed::GetHoveredPin();
  hovered_node_ = ed::GetHoveredNode();
  hovered_link_ = ed::GetHoveredLink();

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
    const auto* ptr = select_node0.AsPointer<NodeBase>();
    show_node_context_menu(select_node0, ptr);
    ImGui::EndPopup();
  }

  if (ImGui::BeginPopup("Link Context Menu")) {
    show_link_context_menu(select_link0, select_link0.AsPointer<InputSocket>());
    ImGui::EndGroup();
  }

  if (ImGui::BeginPopup("Create New Node", ImGuiWindowFlags_AlwaysAutoResize)) {
    show_create_node_window(g);
    ImGui::EndPopup();
  }
  ed::Resume();
}

static void serialize_graph(std::ostream& out) {
  auto& g = ensure_resource<Graph>();
  g.ShrinkToFit();
  Serializer ser(g);
  for (auto const& node :
       g.GetNodes() | ranges::views::filter([](auto const& n) -> bool { return n != nullptr; })) {
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

static void show_pin_descriptor(std::vector<SocketDescriptor>::value_type const& pin_desc) {
  ImGui::Text("Name: %s", pin_desc.Name().c_str());
  ImGui::Text("Desc: %s", pin_desc.Desc().c_str());
  ImGui::Text("Pretty Type: %s", pin_desc.PrettyTypename().c_str());
  ImGui::Text("RAII Type: %s", pin_desc.Type().name());
}

static void draw_hovered() {
  ed::Suspend();
  auto& g = ensure_resource<Graph>();
  if (hovered_pin_) {
    auto p = hovered_pin_.AsPointer<SocketBase>();
    if (p->IsInput()) {
      auto in = static_cast<InputSocket*>(p);
      if (ImGui::BeginTooltip()) {
        ImGui::Text("InputSocket addr: %p", in);
        if (auto nh = g.Get(&(in->Node()))) {
          auto const& node_desc = nh->Node().GetDescriptor();
          auto const& pin_desc = node_desc.GetInputs()[in->Index()];
          show_pin_descriptor(pin_desc);
        }
        ImGui::EndTooltip();
      }
    } else {
      auto out = static_cast<OutputSocket*>(p);
      if (ImGui::BeginTooltip()) {
        ImGui::Text("OutputSocket addr: %p", out);
        if (auto nh = g.Get(&(out->Node()))) {
          auto const& node_desc = nh->Node().GetDescriptor();
          auto const& pin_desc = node_desc.GetOutputs()[out->Index()];
          show_pin_descriptor(pin_desc);
        }
        ImGui::EndTooltip();
      }
    }
  } else if (hovered_node_) {
    if (const auto* node = hovered_node_.AsPointer<NodeBase>()) {
      if (ImGui::BeginTooltip()) {
        if (auto nh = g.Get(node)) {
          ImGui::Text("Name: %s", node->GetDescriptor().Name().c_str());
          ImGui::Text("Description: %s", node->GetDescriptor().Desc().c_str());
          ImGui::Text("Index: %ld", nh->Index());
        }
        ImGui::EndTooltip();
      }
    }
  } else if (hovered_link_) {
    if (auto const* sock = hovered_link_.AsPointer<InputSocket>()) {
      if (ImGui::BeginTooltip()) {
        ImGui::Text("InputSocket addr: %p", sock);
        ImGui::Text("OutputSocket addr: %p", sock->From());
        ImGui::EndTooltip();
      }
    }
  }
  ed::Resume();
}

static void run_once() {
  auto& executor = ensure_executor();
  executor->ExecuteOnce(executor->GetCurrentFrame());
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

  if (auto* node = selected_node_.AsPointer<NodeBase>()) {
    auto const& name = node->GetDescriptor().Name();
    ImGui::SeparatorText(name.c_str());
    if (auto const* renderer = get_custom_node_widget(node->GetDescriptor().Name())) {
      renderer->widget_(node);
    }
  }
  ImGui::End();

  if (need_trigger_event) {
    CacheSequenceUpdateEvent e;
    e.required_frame_id_ = cache_in_show;
    e.is_cleanup_ = false;
    emit_enqueue(e);
    trigger_queue();
  }
}

static void deserialize_graph(Graph& g, std::ifstream& file) try {
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
  d.Deserialize(json.as_object());

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
} catch (std::exception const& e) {
  AX_CRITICAL("Failed to deserialize: {}", e.what());
  g.Clear();
  return;
}

static void do_draw_graph(Graph& g) {
  for (auto const& n :
       g.GetNodes() | ranges::views::filter([](auto const& n) -> bool { return n != nullptr; })) {
    draw_node(n.get());
  }

  for (auto const& n :
       g.GetNodes() | ranges::views::filter([](auto const& n) -> bool { return n != nullptr; })) {
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

  do_draw_graph(g);
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
  draw_hovered();
  ed::End();
  ed::SetCurrentEditor(nullptr);
  ImGui::End();
}

struct GuardAgainstCleanup {
  ~GuardAgainstCleanup() {
    if (context_) {
      fprintf(stderr, "!!! Editor context is not destroyed.\n");
    }
  }
};

static void init(gl::ContextInitEvent const&) {
  AX_TRACE("Create editor context");
  static GuardAgainstCleanup guarding;

  get_internal_node_registry().LoadDefered();
  context_ = ed::CreateEditor();
}

static void cleanup(gl::ContextDestroyEvent const&) {
  AX_TRACE("Destroy editor context");
  ed::DestroyEditor(context_);
  context_ = nullptr;
}

static void on_menu_bar(gl::MainMenuBarRenderEvent) {
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

static void on_tick_logic(gl::TickLogicEvent) {
  // TODO:
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

using NodeRenderMap = std::unordered_map<std::string, CustomNodeRender>;
using WidgetRenderMap = std::unordered_map<std::string, CustomNodeWidget>;

void add_custom_node_render(std::string type, CustomNodeRender const& widget) {
  auto& map = ensure_resource<NodeRenderMap>();
  map.try_emplace(type, widget);
}

CustomNodeRender const* get_custom_node_render(std::string name) {
  auto& map = ensure_resource<NodeRenderMap>();
  auto it = map.find(name);
  if (it != map.end()) {
    return &(it->second);
  }
  return nullptr;
}

void add_custom_node_widget(std::string type, CustomNodeWidget const& widget) {
  auto& map = ensure_resource<WidgetRenderMap>();
  map.try_emplace(type, widget);
}

CustomNodeWidget const* get_custom_node_widget(std::string name) {
  auto& map = ensure_resource<WidgetRenderMap>();
  auto it = map.find(name);
  if (it != map.end()) {
    return &(it->second);
  }
  return nullptr;
}
} // namespace ax::graph