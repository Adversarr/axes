// #include <imgui.h>
// #include "ax/core/init.hpp"
#include "ax/gl/utils.hpp"
// #include "ax/graph/render.hpp"
// #include "ax/nodes/geometry.hpp"
// #include "ax/nodes/gl_prims.hpp"
// #include "ax/nodes/io.hpp"
// #include "ax/nodes/math_types.hpp"
// #include "ax/nodes/stl_types.hpp"
//
// using namespace ax;
// using namespace ax::graph;
//
// int main(int argc, char** argv) {
//   gl::init(argc, argv);
//   graph::install_renderer();
//   nodes::register_stl_types();
//   nodes::register_io_nodes();
//   nodes::register_math_types_nodes();
//   nodes::register_gl_prim_nodes();
//   nodes::register_geometry_nodes();
//
//   gl::enter_main_loop() ;
//   clean_up();
//   return 0;
// }

#include <imgui.h>

#include <iomanip>
#include <iostream>

#include "ax/core/init.hpp"
#include "ax/gl/utils.hpp"
#include "ax/graph/render.hpp"
#include "ax/nodes/stl_types.hpp"

using namespace compute_graph;

class ConstIntegerNode : public NodeDerive<ConstIntegerNode> {
public:
  CG_NODE_COMMON(ConstIntegerNode, "ConstInteger", "Produce a constant integer value");
  CG_NODE_INPUTS();
  CG_NODE_OUTPUTS((int, value, "The constant integer value"));

  static void RenderThis(NodeBase* ptr) {
    ax::graph::begin_draw_node(ptr);
    ax::graph::draw_node_header_default(ptr);
    ImGui::SetNextItemWidth(200);
    ImGui::InputInt("Value", &static_cast<ConstIntegerNode*>(ptr)->value_);
    ax::graph::draw_node_content_default(ptr);
    ax::graph::end_draw_node();
  }

  static void OnRegister() {
    ax::graph::add_custom_node_render(name(), {RenderThis});
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

  static void OnRegister() /* optional */ { printf("Do whatever you want!\n"); }

  void OnConstruct() /* optional */ { std::cout << "Constructing Whatever..." << std::endl; }

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
    std::cout << "EchoString::on_connect" << std::endl;
    std::cout << " has GetInput set? " << std::boolalpha << Has(in::str) << std::endl;
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

  NodeRegistry::instance().LoadDefered();
  NodeHandle<> nh1 = g.PushBack(NodeRegistry::instance().Create<WhateverNode>()),
               nh2 = g.PushBack(NodeRegistry::instance().Create<EchoString>()),
               nh3 = g.PushBack(NodeRegistry::instance().Create<ConstIntegerNode>()),
               nh4 = g.PushBack(NodeRegistry::instance().Create<EchoInteger>()),
               nh5 = g.PushBack(NodeRegistry::instance().Create<ConstantString>()),
               nh6 = g.PushBack(NodeRegistry::instance().Create<ReadContext<std::string>>()),
               nh7 = g.PushBack(NodeRegistry::instance().Create<EchoString>());
  g.Connect(nh1.GetOutput(WhateverNode::out::z), nh2.GetInput(EchoString::in::str));
  g.Connect(nh3.GetOutput(ConstIntegerNode::out::value), nh1.GetInput("x").value());
  g.Connect(nh3.GetOutput(ConstIntegerNode::out::value), nh4.GetInput(0));

  g.Connect(nh5.GetOutput(ConstantString::out::value), nh6.GetInput("key").value());
  g.Connect(nh5.GetOutput(ConstantString::out::value), nh7.GetInput(0));
  g.Connect(nh6.GetOutput(0), nh7.GetInput(0));

  g.TopologySort();

  Context rt;
  rt.PushStack();
  rt.Emplace("what", std::string("is?"));

  for (auto const &node : g.GetNodes()) {
    (*node)(rt);
  }

  ax::gl::enter_main_loop();
  ax::clean_up();
  return 0;
}
