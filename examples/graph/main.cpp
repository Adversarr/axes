// #include <imgui.h>
// #include "ax/core/init.hpp"
#include "ax/gl/utils.hpp"
#include <imgui.h>

#include <iomanip>
#include <iostream>

#include "ax/core/init.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/render.hpp"
#include "ax/math/sparse.hpp"

using namespace compute_graph;

class ConstIntegerNode : public NodeDerive<ConstIntegerNode> {
public:
  CG_NODE_COMMON(ConstIntegerNode, "ConstInteger", "Produce a constant integer value");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((int, value, "The constant integer value"));

  static void RenderThis(NodeBase* ptr) {
    ax::graph::begin_draw_node(ptr);
    ax::graph::draw_node_header_default(ptr);
    ImGui::Text("  Current: %d", static_cast<ConstIntegerNode*>(ptr)->value_);
    ax::graph::draw_node_content_default(ptr);
    ax::graph::end_draw_node();
  }

  static void RenderWidget(NodeBase* ptr) {
    auto* node = static_cast<ConstIntegerNode*>(ptr);
    ImGui::PushID(node);
    ImGui::InputInt("Value", &node->value_);
    ImGui::PopID();
  }

  static void OnRegister() {
    ax::graph::add_custom_node_render(name(), {RenderThis});
    ax::graph::add_custom_node_widget(name(), {RenderWidget});
  }

  void OnConstruct() /* optional */ { Set(out::value, value_); }

  void operator()(Context &) final { SetAll(value_); }
  int value_ {5};
};

class WhateverNode : public NodeDerive<WhateverNode> {
public:
  CG_NODE_COMMON(WhateverNode, "Whatever", "WhateverDescription");
  CG_NODE_INPUTS((int, x, "Describe x", 0 /* default value = 0  */),
                 (int, y, "Describe y", 42 /* default value = 42 */));
  CG_NODE_OUTPUTS((std::string, z, "z equals x + y"));

  void operator()(Context &) final {
    auto x = GetOr(in::x);
    auto y = *GetOr<int>(1);
    Set(out::z, "x=" + std::to_string(x) + ", y=" + std::to_string(y));
  }
};

class EchoString : public NodeDerive<EchoString> {
public:
  CG_NODE_COMMON(EchoString, "EchoString", "EchoStringDescription");
  CG_NODE_INPUTS((std::string, str, "Input string"));
  CG_NODE_OUTPUTS();

  void OnConnectDispatch(in::str_) /* automatically called. */ {
  }

  void operator()(Context &) final {
    auto str = *Get<std::string>(0);
    std::cout << "str: " << std::quoted(str) << std::endl;
  }
};

class EchoInteger : public NodeDerive<EchoInteger> {
public:
  CG_NODE_COMMON(EchoInteger, "EchoInteger", "EchoIntegerDescription");
  CG_NODE_INPUTS((int, x, "Input integer"));
  CG_NODE_OUTPUTS();

  void operator()(Context &) final {
    auto [x] = GetAll();
    std::cout << "x: " << *x << std::endl;
  }
};

class ConstantString : public NodeDerive<ConstantString> {
public:
  CG_NODE_COMMON(ConstantString, "ConstantString", "ConstantString");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((std::string, value, "Return 'what'"));

  void OnConstruct() /* optional */ { Set(out::value, "what"); }

  void operator()(Context &) final { Set(out::value, "what"); }
};

namespace compute_graph {

template <typename T> class ReadContext final : public NodeDerive<ReadContext<T>> {
public:
  CG_NODE_COMMON(ReadContext, "ReadContext", "Extract a value in context");
  CG_NODE_INPUTS((std::string, key, "Variable name"),
                 (bool, top_only, "Only find the variable on the top frame.", true));
  CG_NODE_OUTPUTS((T, value, "Value of the extracted variable"));

  void operator()(Context &ctx) override {
    const bool top_only = GetOr(in::top_only);
    const std::string &key = Ensure(in::key);

    if (top_only) {
      auto const &value = ctx.GetTop(key);
      Set(out::value, std::any_cast<T const &>(value));
    } else {
      auto const &value = ctx.GetTop(key);
      Set(out::value, std::any_cast<T const &>(value));
    }
  }
};

template <typename T> class WriteContext final : public NodeDerive<WriteContext<T>> {
public:
  CG_NODE_COMMON(WriteContext, "WriteContext", "Write a value to current context frame.");
  CG_NODE_INPUTS((std::string, key, "Variable name"), (T, value, "Value of variable to write"));
  CG_NODE_OUTPUTS();

  void operator()(Context &ctx) override {
    auto const &key = Ensure(in::key);
    auto const &value = Ensure(in::value);
    ctx.Emplace(&key, &value);
  }
};

}  // namespace compute_graph

namespace compute_graph {

template class ReadContext<std::string>;

}



int main(int argc, char **argv) {
  Graph g;
  ax::gl::init(argc, argv);
  ax::graph::install_renderer();

  auto& reg = ax::graph::get_internal_node_registry();
  ConstIntegerNode::RegisterTo(reg);
  WhateverNode::RegisterTo(reg);
  EchoString::RegisterTo(reg);
  EchoInteger::RegisterTo(reg);
  ConstantString::RegisterTo(reg);
  ReadContext<std::string>::RegisterTo(reg);

  ax::math::RealSparseMatrix sparse_matrix;

  ax::gl::enter_main_loop();
  ax::clean_up();
  return 0;
}
